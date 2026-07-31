import json
import logging
import math
import os
import subprocess
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path

import numpy as np
import redis
from redis.exceptions import RedisError

from terasim.overlay import traci
from terasim.profiling import add_timing, get_profile, set_value
from terasim.simulator import Simulator

from terasim_nde_nade.adversity import ConstructionAdversity

from .base import BasePlugin, DEFAULT_REDIS_CONFIG

from ..utils import SimulationState, AgentStateSimplified, SUMOSignal, AgentCommand
from ..utils.sumo_lane_geometry import (
    adapt_lookahead_distances_for_compiled_paths,
    blend_lane_change_lookahead,
    compile_lane_shapes,
    extract_next_link_lane_ids,
    find_lookahead_positions_from_compiled_paths,
    project_position_by_sumo_angle,
    reconstruct_position_from_lane_geometry,
    select_route_aware_lane_projection,
)


def interpolate_by_distance(points, step):
    """
    Interpolate a tuple of tuples so that the distance between each point is equal to 'step'.

    Args:
        points (tuple of tuple): Original shape, e.g., ((x1, y1), (x2, y2), ...)
        step (float): Desired distance between points.

    Returns:
        list of list: Interpolated points as [[x, y], ...] with equal spacing.
    """
    points = np.array(points, dtype=np.float32)
    # Compute distances between consecutive points
    deltas = np.diff(points, axis=0)
    seg_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0)
    total_length = cumulative[-1]
    if total_length == 0:
        return [points[0].tolist()]
    # Generate equally spaced distances
    num_points = int(np.floor(total_length / step)) + 1
    distances = np.linspace(0, total_length, num_points)
    # Interpolate x and y separately
    x_interp = np.interp(distances, cumulative, points[:, 0])
    y_interp = np.interp(distances, cumulative, points[:, 1])
    return [[float(x), float(y)] for x, y in zip(x_interp, y_interp)]


def generate_construction_zone_shape(lane_shape, lane_width, direction):
    """
    Generate a construction zone shape based on the lane shape and lane width.
    The first ten points of the lane_shape are offset laterally, with the offset
    gradually changing from direction * lane_width/2 to -direction * lane_width/2.
    The remaining points are offset by a constant -direction * lane_width/2.

    Args:
        lane_shape (list of list): The lane shape as a list of [x, y] points.
        lane_width (float): The width of the lane.
        direction (int): -1 for from left to right, 1 for from right to left.

    Returns:
        list of list: The offset lane shape.
    """
    n = min(10, len(lane_shape))
    construction_zone_shape = []
    for i, pt in enumerate(lane_shape):
        pt = np.array(pt)
        # Compute tangent direction
        if i < len(lane_shape) - 1:
            next_pt = np.array(lane_shape[i + 1])
            dir_vec = next_pt - pt
        else:
            prev_pt = np.array(lane_shape[i - 1])
            dir_vec = pt - prev_pt
        norm = np.linalg.norm(dir_vec)
        if norm == 0:
            dir_vec = np.array([1.0, 0.0])
        else:
            dir_vec = dir_vec / norm
        # Normal vector (perpendicular)
        normal = np.array([-dir_vec[1], dir_vec[0]]) * direction * -1

        # Compute offset
        if i < n:
            # Linear interpolation from +lane_width/2 to -lane_width/2
            alpha = i / (n - 1) if n > 1 else 0
            offset_val = (1 - alpha) * (lane_width / 2) + alpha * (-lane_width / 2)
        else:
            offset_val = - lane_width / 2

        offset_pt = pt + normal * offset_val
        construction_zone_shape.append(offset_pt.tolist())
    return construction_zone_shape


DEFAULT_COSIM_PLUGIN_CONFIG = {
    "name": "terasim_cosim_plugin",
    "priority": {
        "before_env": {
            "start": -90,
            "step": -90,
            "stop": -90,
        },
        "after_env": {
            "start": 90,
            "step": 90,
            "stop": 90,
        },
    },
}


class TeraSimCoSimPlugin(BasePlugin):
    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        redis_config: dict = DEFAULT_REDIS_CONFIG,
        base_dir: str = "output",
        key_expiry=3600,
        auto_run=False,
        enable_viz=False,
        viz_port=8050,
        viz_update_freq=5,
    ):
        """Initialize the Co-Simulation plugin.

        Args:
            simulation_uuid (str): Unique identifier for the simulation instance.
            plugin_config (dict, optional): Configuration for the plugin. Defaults to DEFAULT_COSIM_PLUGIN_CONFIG.
            redis_config (dict, optional): Configuration for the Redis connection. Defaults to DEFAULT_REDIS_CONFIG.
            base_dir (str, optional): Base directory for the log file. Defaults to "output".
            key_expiry (int, optional): Key expiration time in seconds. Defaults to 3600.
            auto_run (bool, optional): Flag to enable auto-run mode. Defaults to False.
            enable_viz (bool, optional): Enable visualization with Streamlit. Defaults to False.
            viz_port (int, optional): Port for Streamlit server. Defaults to 8050.
            viz_update_freq (int, optional): Visualization update frequency. Defaults to 5.
        """
        super().__init__(simulation_uuid, plugin_config, redis_config)
        # Key expiration time in seconds (default: 1 hour)
        self.key_expiry = key_expiry
        self.auto_run = auto_run
        self.base_dir = base_dir

        # Visualization settings
        self.enable_viz = enable_viz
        self.viz_port = viz_port
        self.viz_update_freq = viz_update_freq
        self.viz_process = None

        # Setup logging
        self.logger = self._setup_logger(base_dir)

        # Maintain controlled agents in each step, assuming each agent can be controlled by only one command
        self.controlled_agents_each_step = set()
        self.feedback_observed_speeds = {}
        self.feedback_observed_positions = {}
        self.feedback_source_carla_frames = {}
        self.feedback_lane_change_active_actor_ids = set()
        feedback_actor_value = os.getenv("CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS", "")
        self.ackermann_feedback_actor_ids = {
            actor_id.strip()
            for actor_id in feedback_actor_value.split(",")
            if actor_id.strip()
        }
        self.ackermann_feedback_mode = os.getenv(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE", "off"
        ).strip().lower()
        self.ackermann_feedback_position_mode = os.getenv(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_POSITION_MODE", "moveTo"
        ).strip()
        if self.ackermann_feedback_position_mode not in {"moveTo", "moveToXY"}:
            self.logger.warning(
                "Invalid Ackermann feedback position mode=%s; using moveTo",
                self.ackermann_feedback_position_mode,
            )
            self.ackermann_feedback_position_mode = "moveTo"
        self.ackermann_feedback_move_to_max_distance = max(
            0.0,
            self._parse_float_env(
                "CARLA_COSIM_ACKERMANN_FEEDBACK_MOVE_TO_MAX_DISTANCE", 8.0
            ),
        )
        self.ackermann_feedback_background_move_to_max_distance = (
            self._parse_optional_float(
                os.getenv(
                    "CARLA_COSIM_ACKERMANN_FEEDBACK_BACKGROUND_MOVE_TO_MAX_DISTANCE",
                    "",
                )
            )
        )
        if (
            self.ackermann_feedback_background_move_to_max_distance is not None
            and self.ackermann_feedback_background_move_to_max_distance < 0.0
        ):
            self.logger.warning(
                "Invalid background Ackermann feedback moveTo max distance=%s; "
                "using the common limit",
                self.ackermann_feedback_background_move_to_max_distance,
            )
            self.ackermann_feedback_background_move_to_max_distance = None
        self.ackermann_feedback_move_to_lane_hysteresis = max(
            0.0,
            self._parse_float_env(
                "CARLA_COSIM_ACKERMANN_FEEDBACK_MOVE_TO_LANE_HYSTERESIS", 0.35
            ),
        )
        self.feedback_lane_states = {}
        self.feedback_edge_lane_ids_cache = {}
        self.feedback_lane_geometry_cache = {}
        self.last_agent_command_failure = None
        self.continue_on_ackermann_feedback_failure = self._parse_bool_env(
            "TERASIM_COSIM_CONTINUE_ON_ACKERMANN_FEEDBACK_FAILURE", False
        )
        self.continue_on_background_ackermann_feedback_failure = self._parse_bool_env(
            "TERASIM_COSIM_CONTINUE_ON_BACKGROUND_ACKERMANN_FEEDBACK_FAILURE",
            False,
        )
        if self.continue_on_ackermann_feedback_failure:
            self.logger.warning(
                "Ackermann feedback failures are non-fatal. This mode is intended "
                "for performance measurements only."
            )
        elif self.continue_on_background_ackermann_feedback_failure:
            self.logger.warning(
                "Background Ackermann feedback failures are non-fatal; AV feedback "
                "failures remain fail-closed. This mode is intended for visual and "
                "performance validation only."
            )
        self.ackermann_feedback_lane_index = self._parse_int_env(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_LANE_INDEX", 0
        )
        self.ackermann_feedback_keep_route = self._parse_int_env(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_KEEP_ROUTE", 0
        )
        if self.ackermann_feedback_keep_route not in range(8):
            self.logger.warning(
                "Invalid Ackermann feedback keepRoute=%s; using 0",
                self.ackermann_feedback_keep_route,
            )
            self.ackermann_feedback_keep_route = 0
        self.ackermann_feedback_log_lane_transitions = self._parse_bool_env(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_LOG_LANE_TRANSITIONS", False
        )
        self.ackermann_feedback_lc_keep_right = self._parse_optional_float(
            os.getenv("CARLA_COSIM_ACKERMANN_FEEDBACK_LC_KEEP_RIGHT", "")
        )
        if (
            self.ackermann_feedback_lc_keep_right is not None
            and self.ackermann_feedback_lc_keep_right < 0.0
        ):
            self.logger.warning(
                "Invalid Ackermann feedback lcKeepRight=%s; leaving SUMO default unchanged",
                self.ackermann_feedback_lc_keep_right,
            )
            self.ackermann_feedback_lc_keep_right = None
        lc_keep_right_actor_value = os.getenv(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_LC_KEEP_RIGHT_ACTORS", "AV"
        )
        self.ackermann_feedback_lc_keep_right_actor_ids = {
            actor_id.strip()
            for actor_id in lc_keep_right_actor_value.split(",")
            if actor_id.strip()
        }
        self.ackermann_feedback_lane_change_settings_applied = set()
        default_max_agent_commands = (
            10000
            if self.ackermann_feedback_mode != "off" and self.ackermann_feedback_actor_ids
            else 100
        )
        self.max_agent_commands_per_step = max(
            1,
            int(
                self._parse_float_env(
                    "TERASIM_COSIM_MAX_AGENT_COMMANDS_PER_STEP",
                    default_max_agent_commands,
                )
            ),
        )

        # Cache construction zone shapes
        self.construction_zone_shapes = None

        # Initialize last orientations cache
        self.last_orientations = {}  # {vehicle_id: (last_orientation, last_time)}
        self.lookahead_lane_shape_cache = {}  # {lane_id: immutable lane shape}
        self.lookahead_geometry_cache = {}  # {lane ID tuple: compiled route geometry}
        # Route IDs are identifiers, not immutable route versions, so the full
        # route edge sequence is part of each per-vehicle cache key.
        self.lookahead_vehicle_route_cache = {}
        self.lookahead_straight_min_distance = max(
            0.1, self._parse_float_env("TERASIM_COSIM_LOOKAHEAD_STRAIGHT_MIN_DISTANCE", 7.0)
        )
        self.lookahead_max_distance = max(
            self.lookahead_straight_min_distance,
            self._parse_float_env("TERASIM_COSIM_LOOKAHEAD_MAX_DISTANCE", 15.0),
        )
        self.lookahead_curve_min_distance = max(
            0.1, self._parse_float_env("TERASIM_COSIM_LOOKAHEAD_CURVE_MIN_DISTANCE", 3.5)
        )
        self.lookahead_curve_start_radians = math.radians(
            max(0.0, self._parse_float_env("TERASIM_COSIM_LOOKAHEAD_CURVE_START_DEG", 5.0))
        )
        self.lookahead_curve_full_scale_radians = math.radians(
            max(
                math.degrees(self.lookahead_curve_start_radians) + 0.1,
                self._parse_float_env("TERASIM_COSIM_LOOKAHEAD_CURVE_FULL_SCALE_DEG", 45.0),
            )
        )
        self.lookahead_lane_change_speed_start = max(
            0.0,
            self._parse_float_env(
                "TERASIM_COSIM_LOOKAHEAD_LANE_CHANGE_SPEED_START", 0.05
            ),
        )
        self.lookahead_lane_change_speed_full = max(
            self.lookahead_lane_change_speed_start + 1e-3,
            self._parse_float_env(
                "TERASIM_COSIM_LOOKAHEAD_LANE_CHANGE_SPEED_FULL", 0.35
            ),
        )
        self.lookahead_lane_change_offset_start = max(
            0.0,
            self._parse_float_env(
                "TERASIM_COSIM_LOOKAHEAD_LANE_CHANGE_OFFSET_START", 0.15
            ),
        )
        self.lookahead_lane_change_offset_full = max(
            self.lookahead_lane_change_offset_start + 1e-3,
            self._parse_float_env(
                "TERASIM_COSIM_LOOKAHEAD_LANE_CHANGE_OFFSET_FULL", 0.75
            ),
        )

        # Initialize health monitoring
        self.error_count = 0
        self.last_successful_operation = time.time()

        self.idle_state_write_interval = self._parse_float_env(
            "TERASIM_COSIM_IDLE_STATE_WRITE_INTERVAL", 0.5
        )
        self._last_idle_state_write_wall_time = 0.0

        self.state_filter_enabled = self._parse_bool_env(
            "TERASIM_COSIM_STATE_FILTER", False
        )
        self.state_filter_center_id = os.getenv(
            "TERASIM_COSIM_STATE_FILTER_CENTER_ID", "AV"
        )
        self.state_filter_radius = self._parse_optional_float(
            os.getenv("TERASIM_COSIM_STATE_FILTER_RADIUS", "")
        )
        self.state_filter_missing_center_logged = False
        self.state_filter_error_logged = False
        self.state_detail_radius = self._parse_optional_float(
            os.getenv("TERASIM_COSIM_STATE_DETAIL_RADIUS", "")
        )
        self.state_detail_hysteresis = max(
            0.0,
            self._parse_float_env("TERASIM_COSIM_STATE_DETAIL_HYSTERESIS", 10.0),
        )
        self.state_detail_filter_enabled = bool(
            self.state_detail_radius is not None and self.state_detail_radius > 0.0
        )
        self.state_detail_active_vehicle_ids = set()
        self.vehicle_static_state_cache = {}
        # lon/lat are not consumed by CARLA. Keep them enabled by default for
        # the Redis/gRPC API contract; the in-process runner disables them.
        self.state_export_geo_enabled = self._parse_bool_env(
            "TERASIM_COSIM_STATE_EXPORT_GEO", True
        )
        self.traffic_light_information_cache = {}
        self.traffic_light_id_cache = None
        self._current_vehicle_id_set = set()
        self._current_person_id_set = set()
        self.state_context_subscription_enabled = self._parse_bool_env(
            "TERASIM_COSIM_STATE_CONTEXT_SUBSCRIPTION", True
        )
        self.state_context_subscription_active = False
        self.state_context_subscription_error_logged = False
        self.state_vehicle_context_results = {}
        if self.state_filter_enabled:
            self.logger.info(
                "TeraSim co-sim state filter enabled: center=%s radius=%s",
                self.state_filter_center_id,
                self.state_filter_radius,
            )

        if self.state_detail_filter_enabled:
            self.logger.info(
                "TeraSim detailed state radius enabled: center=%s "
                "enterRadius=%.1fm exitRadius=%.1fm",
                self.state_filter_center_id,
                self.state_detail_radius,
                self.state_detail_radius + self.state_detail_hysteresis,
            )

        self.lane_relative_position_enabled = self._parse_bool_env(
            "TERASIM_COSIM_LANE_RELATIVE_POSITION", False
        )
        if self.lane_relative_position_enabled:
            self.logger.info(
                "TeraSim co-sim lane-relative reconstructed positions enabled "
                "for filtered state vehicles"
            )

    @staticmethod
    def _parse_float_env(name, default):
        value = os.getenv(name)
        if value in (None, ""):
            return default
        try:
            return float(value)
        except ValueError:
            return default

    @staticmethod
    def _parse_int_env(name, default):
        value = os.getenv(name)
        if value in (None, ""):
            return default
        try:
            return int(value)
        except ValueError:
            return default

    @staticmethod
    def _parse_bool_env(name, default=False):
        value = os.getenv(name)
        if value in (None, ""):
            return default
        return value.strip().lower() not in {"0", "false", "no", "off"}

    @staticmethod
    def _parse_optional_float(value):
        if value in (None, ""):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _is_ackermann_feedback_actor(self, actor_id):
        if getattr(self, "ackermann_feedback_mode", "off") == "off":
            return False
        actor_ids = getattr(self, "ackermann_feedback_actor_ids", set())
        return actor_id in actor_ids or (actor_id != "AV" and "*" in actor_ids)

    def _should_continue_after_agent_command_failure(self):
        failure = self.last_agent_command_failure or {}
        reason = str(failure.get("reason", ""))
        is_feedback_failure = bool(failure.get("ackermann_feedback")) or reason.startswith(
            "ackermann_feedback_"
        )
        if not is_feedback_failure:
            return False
        actor_id = str(failure.get("actor_id", ""))
        continue_all = getattr(self, "continue_on_ackermann_feedback_failure", False)
        continue_background = getattr(
            self,
            "continue_on_background_ackermann_feedback_failure",
            False,
        )
        if not continue_all and not (
            continue_background and actor_id and actor_id != "AV"
        ):
            return False
        self.logger.warning(
            "Validation mode: ignoring Ackermann feedback command failure and "
            "continuing SUMO actor=%s reason=%s",
            actor_id,
            reason or "unknown",
        )
        return True

    def _ensure_ackermann_feedback_lane_change_settings(self, actor_id):
        lc_keep_right = getattr(self, "ackermann_feedback_lc_keep_right", None)
        if lc_keep_right is None:
            return

        lc_keep_right_actor_ids = getattr(
            self, "ackermann_feedback_lc_keep_right_actor_ids", {"AV"}
        )
        if actor_id not in lc_keep_right_actor_ids and "*" not in lc_keep_right_actor_ids:
            return

        applied_actor_ids = getattr(
            self,
            "ackermann_feedback_lane_change_settings_applied",
            set(),
        )
        if actor_id in applied_actor_ids:
            return

        parameter_name = "laneChangeModel.lcKeepRight"
        parameter_value = f"{lc_keep_right:g}"
        traci.vehicle.setParameter(actor_id, parameter_name, parameter_value)
        applied_value = traci.vehicle.getParameter(actor_id, parameter_name)
        try:
            matches_requested_value = float(applied_value) == lc_keep_right
        except (TypeError, ValueError):
            matches_requested_value = False
        if not matches_requested_value:
            raise RuntimeError(
                f"SUMO did not apply {parameter_name}={parameter_value} "
                f"to {actor_id}; reported {applied_value!r}"
            )

        applied_actor_ids.add(actor_id)
        self.ackermann_feedback_lane_change_settings_applied = applied_actor_ids
        self.logger.info(
            "Ackermann feedback lane-change settings applied actor=%s %s=%s",
            actor_id,
            parameter_name,
            applied_value,
        )

    def _read_ackermann_feedback_lane_state(self, actor_id):
        try:
            return {
                "road_id": traci.vehicle.getRoadID(actor_id),
                "lane_id": traci.vehicle.getLaneID(actor_id),
                "lane_position": traci.vehicle.getLanePosition(actor_id),
                "route_index": traci.vehicle.getRouteIndex(actor_id),
            }
        except Exception as exc:
            self.logger.debug(
                "Could not read Ackermann feedback lane state for %s: %s",
                actor_id,
                exc,
            )
            return None

    def _log_ackermann_feedback_lane_transition(self, actor_id, source, before, after):
        if before is None or after is None or before["lane_id"] == after["lane_id"]:
            return
        self.logger.warning(
            "Ackermann feedback lane transition actor=%s source=%s "
            "road=%s->%s lane=%s->%s lanePos=%s->%s routeIndex=%s->%s",
            actor_id,
            source,
            before["road_id"],
            after["road_id"],
            before["lane_id"],
            after["lane_id"],
            before["lane_position"],
            after["lane_position"],
            before["route_index"],
            after["route_index"],
        )

    @staticmethod
    def _append_unique_lane_id(lane_ids, seen_lane_ids, lane_id):
        if lane_id and lane_id not in seen_lane_ids:
            lane_ids.append(lane_id)
            seen_lane_ids.add(lane_id)

    def _append_edge_lane_ids(self, lane_ids, seen_lane_ids, edge_id):
        if not edge_id:
            return
        edge_lane_ids_cache = getattr(self, "feedback_edge_lane_ids_cache", None)
        if edge_lane_ids_cache is None:
            edge_lane_ids_cache = {}
            self.feedback_edge_lane_ids_cache = edge_lane_ids_cache
        edge_lane_ids = edge_lane_ids_cache.get(edge_id)
        if edge_lane_ids is None:
            try:
                lane_count = traci.edge.getLaneNumber(edge_id)
            except Exception:
                return
            edge_lane_ids = tuple(f"{edge_id}_{lane_index}" for lane_index in range(lane_count))
            edge_lane_ids_cache[edge_id] = edge_lane_ids
        for lane_id in edge_lane_ids:
            self._append_unique_lane_id(lane_ids, seen_lane_ids, lane_id)

    def _get_ackermann_feedback_lane_candidates(self, actor_id):
        """Return route-compatible lanes near an Ackermann feedback actor."""
        lane_ids = []
        seen_lane_ids = set()
        current_lane_id = traci.vehicle.getLaneID(actor_id)
        current_road_id = traci.vehicle.getRoadID(actor_id)
        route = traci.vehicle.getRoute(actor_id)
        route_index = traci.vehicle.getRouteIndex(actor_id)

        self._append_unique_lane_id(lane_ids, seen_lane_ids, current_lane_id)
        self._append_edge_lane_ids(lane_ids, seen_lane_ids, current_road_id)

        try:
            next_links = traci.vehicle.getNextLinks(actor_id)
        except Exception:
            next_links = []
        for lane_id in extract_next_link_lane_ids(next_links[:1]):
            self._append_unique_lane_id(lane_ids, seen_lane_ids, lane_id)

        if route:
            route_start = max(0, route_index)
            for edge_id in route[route_start : route_start + 2]:
                self._append_edge_lane_ids(lane_ids, seen_lane_ids, edge_id)

        lane_geometry_cache = getattr(self, "feedback_lane_geometry_cache", None)
        if lane_geometry_cache is None:
            lane_geometry_cache = {}
            self.feedback_lane_geometry_cache = lane_geometry_cache
        candidates = []
        for lane_id in lane_ids:
            geometry = lane_geometry_cache.get(lane_id)
            if geometry is None:
                try:
                    geometry = {
                        "lane_id": lane_id,
                        "shape": traci.lane.getShape(lane_id),
                        "length": traci.lane.getLength(lane_id),
                    }
                except Exception:
                    continue
                lane_geometry_cache[lane_id] = geometry
            candidates.append(geometry)
        return current_lane_id, candidates

    @staticmethod
    def _profile_feedback_command_call(
        profile_ctx, category, command_name, function, *args, **kwargs
    ):
        """Call one feedback operation and record its wall time while profiling."""
        if get_profile(profile_ctx) is None:
            return function(*args, **kwargs)

        start = time.perf_counter()
        try:
            return function(*args, **kwargs)
        finally:
            elapsed = time.perf_counter() - start
            base = f"terasim_internal.feedback_command_breakdown.{category}"
            add_timing(profile_ctx, f"{base}.total_s", elapsed)
            add_timing(profile_ctx, f"{base}.{command_name}_s", elapsed)
            add_timing(profile_ctx, f"{base}.{command_name}_calls", 1.0)

    def _get_ackermann_feedback_current_lane_candidate(self, actor_id, profile_ctx=None):
        """Return only the SUMO lane that moveTo is allowed to preserve."""
        current_lane_id = self._profile_feedback_command_call(
            profile_ctx,
            "traci",
            "vehicle_get_lane_id",
            traci.vehicle.getLaneID,
            actor_id,
        )
        if not current_lane_id:
            return current_lane_id, []

        lane_geometry_cache = getattr(self, "feedback_lane_geometry_cache", None)
        if lane_geometry_cache is None:
            lane_geometry_cache = {}
            self.feedback_lane_geometry_cache = lane_geometry_cache
        geometry = lane_geometry_cache.get(current_lane_id)
        if geometry is None:
            shape = self._profile_feedback_command_call(
                profile_ctx,
                "traci",
                "lane_get_shape",
                traci.lane.getShape,
                current_lane_id,
            )
            length = self._profile_feedback_command_call(
                profile_ctx,
                "traci",
                "lane_get_length",
                traci.lane.getLength,
                current_lane_id,
            )
            geometry = {
                "lane_id": current_lane_id,
                "shape": shape,
                "length": length,
            }
            lane_geometry_cache[current_lane_id] = geometry
            add_timing(
                profile_ctx,
                "terasim_internal.feedback_command_breakdown.lane_geometry_cache_misses",
                1.0,
            )
        else:
            add_timing(
                profile_ctx,
                "terasim_internal.feedback_command_breakdown.lane_geometry_cache_hits",
                1.0,
            )
        return current_lane_id, [geometry]

    def _move_ackermann_feedback_actor(
        self, actor_id, position, sumo_angle, profile_ctx=None
    ):
        current_lane_id, current_lane_candidates = (
            self._get_ackermann_feedback_current_lane_candidate(actor_id, profile_ctx)
        )
        # moveTo cannot represent CARLA lateral offset. Let SUMO own lane
        # changes and only update longitudinal progress on its current lane.
        max_distance = getattr(
            self,
            "ackermann_feedback_move_to_max_distance",
            8.0,
        )
        if actor_id != "AV":
            background_max_distance = getattr(
                self,
                "ackermann_feedback_background_move_to_max_distance",
                None,
            )
            if background_max_distance is not None:
                max_distance = background_max_distance

        projection = self._profile_feedback_command_call(
            profile_ctx,
            "python",
            "current_lane_projection",
            select_route_aware_lane_projection,
            position,
            sumo_angle,
            current_lane_candidates,
            current_lane_id=current_lane_id,
            lane_switch_hysteresis=getattr(
                self,
                "ackermann_feedback_move_to_lane_hysteresis",
                0.35,
            ),
            max_distance=max_distance,
            prefer_current_lane=True,
        )
        if projection is None:
            candidate_lane_ids = [
                candidate["lane_id"] for candidate in current_lane_candidates
            ]
            self.logger.error(
                "Ackermann feedback moveTo mapping failed actor=%s "
                "position=(%.3f, %.3f) angle=%.3f currentLane=%s candidates=%s",
                actor_id,
                position[0],
                position[1],
                sumo_angle,
                current_lane_id,
                candidate_lane_ids,
            )
            self.last_agent_command_failure = {
                "actor_id": actor_id,
                "reason": "ackermann_feedback_moveTo_mapping_failed",
                "ackermann_feedback": True,
                "position": [position[0], position[1]],
                "sumo_angle": sumo_angle,
                "current_lane_id": current_lane_id,
                "candidate_lane_ids": candidate_lane_ids,
            }
            return None

        self._profile_feedback_command_call(
            profile_ctx,
            "traci",
            "vehicle_move_to",
            traci.vehicle.moveTo,
            actor_id,
            projection["lane_id"],
            projection["lane_position"],
        )
        self.logger.debug(
            "Ackermann feedback moveTo actor=%s lane=%s lanePos=%.3f "
            "distance=%.3f headingError=%.3f",
            actor_id,
            projection["lane_id"],
            projection["lane_position"],
            projection["distance"],
            projection["heading_error"],
        )
        return projection

    def _setup_logger(self, base_dir: str) -> logging.Logger:
        """Setup logger for the plugin.

        Args:
            base_dir (str): Base directory for the log file.

        Returns:
            logging.Logger: Logger instance for the plugin.
        """
        logger = logging.getLogger(f"{self.plugin_name}-{self.simulation_uuid}")
        logger.setLevel(logging.DEBUG)

        # Create a rotating file handler
        file_handler = RotatingFileHandler(
            f"{base_dir}/{self.plugin_name}.log",
            maxBytes=10 * 1024 * 1024,  # 10MB
            backupCount=5,
        )
        file_handler.setLevel(logging.DEBUG)

        # Create console handler
        console_handler = logging.StreamHandler()
        console_level_name = os.getenv("TERASIM_COSIM_CONSOLE_LOG_LEVEL", "INFO").upper()
        console_handler.setLevel(getattr(logging, console_level_name, logging.INFO))

        # Create formatter and add it to the handlers
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Add the handlers to the logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger

    def function_before_env_start(self, simulator: Simulator, ctx):
        """Connect to the Redis server and set the simulation status to be 'initializing'.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Initialize Redis connection
            self.redis_client = redis.Redis(**self.redis_config)

            # Clear old data and set initial state with expiration
            self.redis_client.delete(f"simulation:{self.simulation_uuid}:*")
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:status", "initializing", ex=self.key_expiry
            )

            self.logger.info(
                f"Redis connection established. Simulation UUID: {self.simulation_uuid}, start initialization!"
            )

            # Add this line to write initial simulation state
            # self._write_simulation_state(simulator)

            return True
        except RedisError as e:
            self.logger.error(f"Failed to initialize Redis: {e}")
            return False
        except Exception as e:
            self.logger.exception(f"Unexpected error during initialization: {e}")
            return False
        
    def function_after_env_start(self, simulator: Simulator, ctx):
        """Set the simulation status to 'wait_for_tick' after finishing the intialization.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Set initial state with expiration
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:status", "wait_for_tick", ex=self.key_expiry
            )

            self.logger.info(
                f"Redis connection established. Simulation UUID: {self.simulation_uuid}, finish initialization!"
            )

            # Extract map data and start visualization if enabled
            if self.enable_viz:
                self.logger.info("Visualization enabled, extracting map data...")
                map_data = self._extract_map_geometry(simulator.sumo_net)
                
                # Store map data in Redis
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:map_data",
                    json.dumps(map_data),
                    ex=self.key_expiry
                )
                
                self.logger.info(
                    f"Map data extracted: {len(map_data['lanes'])} lanes, "
                    f"{len(map_data['junctions'])} junctions, "
                    f"{len(map_data['traffic_lights'])} traffic lights"
                )
                
                # Start Streamlit visualization
                self._start_streamlit_service()

            return True
        except RedisError as e:
            self.logger.error(f"Failed to initialize Redis: {e}")
            return False
        except Exception as e:
            self.logger.exception(f"Unexpected error during initialization: {e}")
            return False

    def function_before_env_step(self, simulator: Simulator, ctx):
        """Handle simulation step logic, including handling simulation level commands, handling agent-level command, and retrieving simulation states.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        
        Returns:
            bool: True if the simulation step was successful, False otherwise.
        """
        idle_start_time = time.time()
        
        while True:
            # Auto-stop if no commands for 10 minutes
            if time.time() - idle_start_time > 600:  # 10 minutes
                self.logger.warning("No activity for 10 minutes, auto-stopping")
                return False
                
            # Handle simulation control commands
            command = self._get_and_handle_command(simulator)
            if command == "stop":
                return False

            if command:
                idle_start_time = time.time()  # Reset idle timer

            # Handle all pending vehicle commands
            self.controlled_agents_each_step.clear()
            if not self._handle_pending_agent_commands(simulator):
                return False

            now = time.time()
            should_write_idle_state = (
                self.auto_run
                or self._last_idle_state_write_wall_time <= 0.0
                or (
                    self.idle_state_write_interval > 0.0
                    and now - self._last_idle_state_write_wall_time
                    >= self.idle_state_write_interval
                )
            )
            if should_write_idle_state:
                state_write_success = self._write_simulation_state(simulator)
                if not state_write_success:
                    return False
                self._last_idle_state_write_wall_time = now

            if self._is_simulation_paused():
                time.sleep(0.1)  # Wait while paused
                continue

            if not self.auto_run:
                if command == "tick":
                    break  # Proceed with the simulation step
                else:
                    time.sleep(0.005)  # Short sleep to prevent busy waiting
                    continue

            break  # Proceed with the simulation step in auto_run mode
        self.redis_client.set(
            f"simulation:{self.simulation_uuid}:status", "running", ex=self.key_expiry
        )
        self.logger.info("Simulation step started")
        return True
    
    def function_after_env_step(self, simulator: Simulator, ctx):
        """Handle post-simulation step logic, including updating simulation status.
        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        Returns:
            bool: True if the simulation step was successful, False otherwise.
        """
        state_write_success = self._write_simulation_state(simulator)
        if not state_write_success:
            return False

        completed_sumo_time = traci.simulation.getTime()
        self.redis_client.set(
            f"simulation:{self.simulation_uuid}:completed_sumo_time",
            completed_sumo_time,
            ex=self.key_expiry,
        )
        completed_tick_count = self.redis_client.incr(
            f"simulation:{self.simulation_uuid}:completed_tick_count"
        )
        self.redis_client.expire(
            f"simulation:{self.simulation_uuid}:completed_tick_count", self.key_expiry
        )
        self.redis_client.set(
            f"simulation:{self.simulation_uuid}:status", "ticked", ex=self.key_expiry
        )
        self._last_idle_state_write_wall_time = time.time()
        self.logger.info(
            "Simulation step finished! completed_sumo_time=%s completed_tick_count=%s",
            completed_sumo_time,
            completed_tick_count,
        )
        return True

    def function_before_env_stop(self, simulator: Simulator, ctx):
        """Handle simulation stopping logic. Default implementation does nothing.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        pass

    def function_after_env_stop(self, simulator: Simulator, ctx):
        """Handle post-simulation stopping logic, including updating simulation status.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Stop visualization if enabled
            if self.enable_viz and self.viz_process:
                self.logger.info("Stopping visualization service...")
                try:
                    self.viz_process.terminate()
                    self.viz_process.wait(timeout=5)
                    self.logger.info("Visualization service stopped")
                except Exception as e:
                    self.logger.error(f"Error stopping visualization: {e}")
                    
            if self.redis_client:
                finish_string = f"Simulation {self.simulation_uuid} finished!"
                # Set simulation end status briefly before cleanup
                results_dict = {
                    "finish_reason": simulator.env.record.get("finish_reason",""),
                    "collider": simulator.env.record.get("collider",""),
                    "victim": simulator.env.record.get("victim",""),
                }
                results_str = json.dumps(results_dict)
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:status",
                    "finished",
                    ex=1800,  # Keep status for 10 seconds only
                )
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:result",
                    results_str,
                    ex=1800,  # Keep status for 30 minutes
                )

                # Clean up visualization data if enabled
                if self.enable_viz:
                    self.redis_client.delete(f"simulation:{self.simulation_uuid}:map_data")

                # Close Redis connection
                self.redis_client.close()
                self.logger.info(finish_string)
        except RedisError as e:
            self.logger.error(f"Error during Redis cleanup: {e}")
        except Exception as e:
            self.logger.exception(f"Unexpected error during cleanup: {e}")
    
    def inject(self, simulator: Simulator, ctx):
        """Inject the plugin into the simulation.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook(f"{self.plugin_name}_before_env_start", self.function_before_env_start, priority=self.plugin_priority["before_env"]["start"])
        simulator.start_pipeline.hook(f"{self.plugin_name}_after_env_start", self.function_after_env_start, priority=self.plugin_priority["after_env"]["start"])
        simulator.step_pipeline.hook(f"{self.plugin_name}_before_env_step", self.function_before_env_step, priority=self.plugin_priority["before_env"]["step"])
        simulator.step_pipeline.hook(f"{self.plugin_name}_after_env_step", self.function_after_env_step, priority=self.plugin_priority["after_env"]["step"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_before_env_stop", self.function_before_env_stop, priority=self.plugin_priority["before_env"]["stop"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_after_env_stop", self.function_after_env_stop, priority=self.plugin_priority["after_env"]["stop"])
    
    def _check_simulation_status(self) -> bool:
        """Check if simulation is still running.

        Returns:
            bool: True if simulation is running, False if stopped or doesn't exist
        """
        status = self.redis_client.get(f"simulation:{self.simulation_uuid}:status")
        if not status or status.decode("utf-8") == "finished":
            self.logger.warning(
                f"Simulation {self.simulation_uuid} is stopped or doesn't exist"
            )
            return False
        return True

    def _get_and_handle_command(self, simulator: Simulator) -> str | None:
        """Get and handle simulation control commands.

        Args:
            simulator (Simulator): The simulator object.

        Returns:
            str | None: The control command to execute, or None if no command is present.
        """
        if not self._check_simulation_status():
            return "stop"
        command = self.redis_client.get(f"simulation:{self.simulation_uuid}:control")
        if command:
            command = command.decode("utf-8")
            self._handle_control_command(command, simulator)
            if command != "stop":
                self.redis_client.delete(f"simulation:{self.simulation_uuid}:control")
        return command

    def _is_simulation_paused(self) -> bool:
        """Check if the simulation is paused.

        Returns:
            bool: True if simulation is paused, False otherwise.
        """
        if not self._check_simulation_status():
            return False
        return bool(self.redis_client.exists(f"simulation:{self.simulation_uuid}:paused"))

    def _handle_control_command(self, command, simulator):
        """Handle simulation control commands.
        
        Args:
            command (str): The control command to execute.
            simulator (Simulator): The simulator object.
        """
        if command == "pause":
            self.redis_client.set(f"simulation:{self.simulation_uuid}:paused", "1")
            self.logger.info("Simulation paused")
        elif command == "resume":
            self.redis_client.delete(f"simulation:{self.simulation_uuid}:paused")
            self.logger.info("Simulation resumed")
        elif command == "stop":
            self.logger.info("Stopping simulation")
            simulator.running = False
        # Add more control command handling logic as needed

    def get_vehicle_vru_ids(self):
        """Get all vehicle and VRU IDs and retain the per-step type sets."""
        vehicle_id_list = traci.vehicle.getIDList()
        person_id_list = traci.person.getIDList()
        self._current_vehicle_id_set = set(vehicle_id_list)
        self._current_person_id_set = set(person_id_list)
        all_ids = list(self._current_vehicle_id_set | self._current_person_id_set)
        # Separate by type: construction objects, VRUs, and regular vehicles
        construction_ids = [id for id in all_ids if id.startswith("CONSTRUCTION_")]
        vru_ids = [id for id in all_ids if "VRU" in id and id not in construction_ids]
        vehicle_ids = [id for id in all_ids if id not in vru_ids and id not in construction_ids]
        return vehicle_ids, vru_ids, construction_ids

    @staticmethod
    def _state_context_subscription_variables():
        """Return the vehicle variables shared by NADE and state export."""
        return [
            traci.constants.VAR_DISTANCE,
            traci.constants.VAR_POSITION,
            traci.constants.VAR_POSITION3D,
            traci.constants.VAR_ANGLE,
            traci.constants.VAR_SPEED,
            traci.constants.VAR_SPEED_LAT,
            traci.constants.VAR_ACCELERATION,
            traci.constants.VAR_LANE_ID,
            traci.constants.VAR_LANEPOSITION,
            traci.constants.VAR_LANEPOSITION_LAT,
            traci.constants.VAR_ROUTE_ID,
            traci.constants.VAR_EDGES,
        ]

    def _get_state_vehicle_context_results(self, vehicle_ids):
        """Return AV-centred vehicle state without per-vehicle TraCI queries."""
        self.state_vehicle_context_results = {}
        if (
            not getattr(self, "state_context_subscription_enabled", True)
            or not self.state_filter_enabled
            or self.state_filter_radius is None
            or self.state_filter_radius <= 0
        ):
            return None

        center_id = self.state_filter_center_id
        if center_id not in vehicle_ids:
            self.state_context_subscription_active = False
            return None

        variables = self._state_context_subscription_variables()
        required_variables = {
            traci.constants.VAR_POSITION3D,
            traci.constants.VAR_ANGLE,
            traci.constants.VAR_SPEED,
        }

        try:
            results = None
            if getattr(self, "state_context_subscription_active", False):
                results = traci.vehicle.getContextSubscriptionResults(center_id)
                center_values = (results or {}).get(center_id, {})
                if not required_variables.issubset(center_values):
                    self.state_context_subscription_active = False

            if not getattr(self, "state_context_subscription_active", False):
                traci.vehicle.subscribeContext(
                    center_id,
                    traci.constants.CMD_GET_VEHICLE_VARIABLE,
                    self.state_filter_radius,
                    variables,
                )
                self.state_context_subscription_active = True
                results = traci.vehicle.getContextSubscriptionResults(center_id)

            results = dict(results or {})
            center_values = results.get(center_id, {})
            if not required_variables.issubset(center_values):
                raise RuntimeError("context subscription returned incomplete AV state")

            self.state_context_subscription_error_logged = False
            self.state_vehicle_context_results = results
            return results
        except Exception as exc:
            self.state_context_subscription_active = False
            if not getattr(self, "state_context_subscription_error_logged", False):
                self.logger.warning(
                    "State context subscription failed; falling back to per-vehicle queries: %s",
                    exc,
                )
                self.state_context_subscription_error_logged = True
            return None

    def _filter_vehicle_ids_for_state(self, vehicle_ids):
        if (
            not self.state_filter_enabled
            or self.state_filter_radius is None
            or self.state_filter_radius <= 0
        ):
            return vehicle_ids, {}

        try:
            if self.state_filter_center_id not in vehicle_ids:
                if not self.state_filter_missing_center_logged:
                    self.logger.warning(
                        "State filter center vehicle %s is missing; writing all vehicles",
                        self.state_filter_center_id,
                    )
                    self.state_filter_missing_center_logged = True
                return vehicle_ids, {}

            context_results = self._get_state_vehicle_context_results(vehicle_ids)
            if context_results is not None:
                filtered_vehicle_ids = []
                position_cache = {}
                for vehicle_id in vehicle_ids:
                    values = context_results.get(vehicle_id)
                    if values is None:
                        continue
                    position = values.get(traci.constants.VAR_POSITION3D)
                    if position is None:
                        continue
                    filtered_vehicle_ids.append(vehicle_id)
                    position_cache[vehicle_id] = position
                if self.state_filter_center_id in position_cache:
                    self.state_filter_missing_center_logged = False
                    self.state_filter_error_logged = False
                    return filtered_vehicle_ids, position_cache

            center_position = traci.vehicle.getPosition3D(self.state_filter_center_id)
            position_cache = {self.state_filter_center_id: center_position}
            radius_sq = self.state_filter_radius * self.state_filter_radius
            filtered_vehicle_ids = []
            for vid in vehicle_ids:
                if vid in position_cache:
                    position = position_cache[vid]
                else:
                    position = traci.vehicle.getPosition3D(vid)
                    position_cache[vid] = position
                dx = position[0] - center_position[0]
                dy = position[1] - center_position[1]
                if vid == self.state_filter_center_id or dx * dx + dy * dy <= radius_sq:
                    filtered_vehicle_ids.append(vid)

            self.state_filter_missing_center_logged = False
            self.state_filter_error_logged = False
            return filtered_vehicle_ids, position_cache
        except Exception as e:
            if not self.state_filter_error_logged:
                self.logger.warning("State filter failed; writing all vehicles: %s", e)
                self.state_filter_error_logged = True
            return vehicle_ids, {}

    def _update_state_detail_active_vehicle_ids(self, vehicle_ids, position_cache):
        vehicle_ids = set(vehicle_ids)
        if not getattr(self, "state_detail_filter_enabled", False):
            self.state_detail_active_vehicle_ids = vehicle_ids
            return vehicle_ids

        center_id = self.state_filter_center_id
        center_position = position_cache.get(center_id)
        if center_position is None and center_id in vehicle_ids:
            try:
                center_position = traci.vehicle.getPosition3D(center_id)
                position_cache[center_id] = center_position
            except Exception:
                center_position = None
        if center_position is None:
            # Preserve the full state contract until the AV exists.
            self.state_detail_active_vehicle_ids = vehicle_ids
            return vehicle_ids

        previous_ids = getattr(self, "state_detail_active_vehicle_ids", set())
        enter_radius_sq = self.state_detail_radius * self.state_detail_radius
        exit_radius_sq = (
            self.state_detail_radius + self.state_detail_hysteresis
        ) ** 2
        detail_ids = {center_id, "AV"} & vehicle_ids
        for vehicle_id in vehicle_ids - detail_ids:
            position = position_cache.get(vehicle_id)
            if position is None:
                try:
                    position = traci.vehicle.getPosition3D(vehicle_id)
                    position_cache[vehicle_id] = position
                except Exception:
                    continue
            dx = position[0] - center_position[0]
            dy = position[1] - center_position[1]
            radius_sq = exit_radius_sq if vehicle_id in previous_ids else enter_radius_sq
            if dx * dx + dy * dy <= radius_sq:
                detail_ids.add(vehicle_id)
        self.state_detail_active_vehicle_ids = detail_ids
        return detail_ids

    def _populate_vehicle_static_state(self, vehicle_id, vehicle_state):
        cache = getattr(self, "vehicle_static_state_cache", None)
        if cache is None:
            cache = {}
            self.vehicle_static_state_cache = cache
        static_state = cache.get(vehicle_id)
        if static_state is None:
            static_state = (
                traci.vehicle.getLength(vehicle_id),
                traci.vehicle.getWidth(vehicle_id),
                traci.vehicle.getHeight(vehicle_id),
                traci.vehicle.getTypeID(vehicle_id),
            )
            cache[vehicle_id] = static_state
        (
            vehicle_state.length,
            vehicle_state.width,
            vehicle_state.height,
            vehicle_state.type,
        ) = static_state

    @staticmethod
    def _profile_detail_traci_call(profile_ctx, command_name, function, *args):
        """Call one TraCI command and record its wall time and invocation count."""
        if get_profile(profile_ctx) is None:
            return function(*args)

        start = time.perf_counter()
        try:
            return function(*args)
        finally:
            elapsed = time.perf_counter() - start
            base = "terasim_internal.state_export.ackermann_detail_breakdown.traci"
            add_timing(profile_ctx, f"{base}.total_s", elapsed)
            add_timing(profile_ctx, f"{base}.{command_name}_s", elapsed)
            add_timing(profile_ctx, f"{base}.{command_name}_calls", 1.0)

    @staticmethod
    def _profile_detail_python_call(profile_ctx, operation_name, function, *args):
        """Call pure Python geometry and time it only while profiling is active."""
        if get_profile(profile_ctx) is None:
            return function(*args)

        start = time.perf_counter()
        try:
            return function(*args)
        finally:
            elapsed = time.perf_counter() - start
            base = "terasim_internal.state_export.ackermann_detail_breakdown.python"
            add_timing(profile_ctx, f"{base}.total_s", elapsed)
            add_timing(profile_ctx, f"{base}.{operation_name}_s", elapsed)

    @staticmethod
    def _context_vehicle_value(context_values, constant_name):
        constant = getattr(traci.constants, constant_name, None)
        if constant is None or constant not in context_values:
            return False, None
        return True, context_values[constant]

    def _get_lookahead_lane_shape(self, lane_id, profile_ctx=None):
        if not lane_id:
            return None
        cache = getattr(self, "lookahead_lane_shape_cache", None)
        if cache is None:
            cache = {}
            self.lookahead_lane_shape_cache = cache
        if lane_id in cache:
            return cache[lane_id]
        shape = self._profile_detail_traci_call(
            profile_ctx,
            "lane_get_shape",
            traci.lane.getShape,
            lane_id,
        )
        if not shape or len(shape) < 2:
            return None
        immutable_shape = tuple(tuple(point) for point in shape)
        cache[lane_id] = immutable_shape
        return immutable_shape

    def _populate_lane_relative_position(
        self,
        vehicle_id,
        vehicle_state,
        context_values=None,
        profile_ctx=None,
    ):
        if not self.lane_relative_position_enabled:
            return

        context_values = context_values or {}
        has_lane_id, lane_id = self._context_vehicle_value(
            context_values, "VAR_LANE_ID"
        )
        if not has_lane_id:
            lane_id = self._profile_detail_traci_call(
                profile_ctx,
                "vehicle_get_lane_id",
                traci.vehicle.getLaneID,
                vehicle_id,
            )
        if not lane_id:
            return

        has_lane_position, lane_position = self._context_vehicle_value(
            context_values, "VAR_LANEPOSITION"
        )
        if not has_lane_position:
            lane_position = self._profile_detail_traci_call(
                profile_ctx,
                "vehicle_get_lane_position",
                traci.vehicle.getLanePosition,
                vehicle_id,
            )
        has_lateral_offset, lateral_offset = self._context_vehicle_value(
            context_values, "VAR_LANEPOSITION_LAT"
        )
        if not has_lateral_offset:
            lateral_offset = self._profile_detail_traci_call(
                profile_ctx,
                "vehicle_get_lateral_lane_position",
                traci.vehicle.getLateralLanePosition,
                vehicle_id,
            )
        lane_shape = self._get_lookahead_lane_shape(lane_id, profile_ctx=profile_ctx)
        reconstructed = self._profile_detail_python_call(
            profile_ctx,
            "reconstruct_lane_relative_position",
            reconstruct_position_from_lane_geometry,
            lane_shape,
            lane_position,
            lateral_offset,
            vehicle_state.z,
        )

        vehicle_state.lane_id = lane_id
        vehicle_state.lane_position = lane_position
        vehicle_state.lateral_offset = lateral_offset
        if reconstructed is None:
            return
        (
            vehicle_state.reconstructed_x,
            vehicle_state.reconstructed_y,
            vehicle_state.reconstructed_z,
        ) = reconstructed
        vehicle_state.reconstructed_position_valid = True

    def _lookahead_distance(self, speed):
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        minimum = getattr(self, "lookahead_straight_min_distance", 7.0)
        maximum = getattr(self, "lookahead_max_distance", 15.0)
        return min(maximum, max(minimum, speed))

    def _get_vehicle_lookahead_compiled_path(
        self,
        vehicle_id,
        context_values=None,
        profile_ctx=None,
    ):
        context_values = context_values or {}
        has_current_lane, current_lane = self._context_vehicle_value(
            context_values, "VAR_LANE_ID"
        )
        if not has_current_lane:
            try:
                current_lane = self._profile_detail_traci_call(
                    profile_ctx,
                    "vehicle_get_lane_id",
                    traci.vehicle.getLaneID,
                    vehicle_id,
                )
            except Exception:
                current_lane = ""

        has_next_links, next_links = self._context_vehicle_value(
            context_values, "VAR_NEXT_LINKS"
        )
        if has_next_links:
            next_lane_ids = self._profile_detail_python_call(
                profile_ctx,
                "extract_next_link_lane_ids",
                extract_next_link_lane_ids,
                next_links,
            )
        elif current_lane:
            vehicle_route_cache = getattr(self, "lookahead_vehicle_route_cache", None)
            if vehicle_route_cache is None:
                vehicle_route_cache = {}
                self.lookahead_vehicle_route_cache = vehicle_route_cache
            lane_route_cache = vehicle_route_cache.setdefault(vehicle_id, {})
            has_route_edges, route_edges = self._context_vehicle_value(
                context_values, "VAR_EDGES"
            )
            route_signature = tuple(route_edges or ()) if has_route_edges else None
            route_cache_key = (
                (current_lane, route_signature) if route_signature is not None else None
            )
            next_lane_ids = (
                lane_route_cache.get(route_cache_key)
                if route_cache_key is not None
                else None
            )
            if next_lane_ids is None:
                try:
                    next_links = self._profile_detail_traci_call(
                        profile_ctx,
                        "vehicle_get_next_links",
                        traci.vehicle.getNextLinks,
                        vehicle_id,
                    )
                except Exception:
                    next_links = []
                next_lane_ids = tuple(
                    self._profile_detail_python_call(
                        profile_ctx,
                        "extract_next_link_lane_ids",
                        extract_next_link_lane_ids,
                        next_links,
                    )
                )
                if route_cache_key is not None:
                    lane_route_cache[route_cache_key] = next_lane_ids
        else:
            next_lane_ids = ()

        lane_ids = []
        for lane_id in [current_lane, *next_lane_ids]:
            if lane_id and lane_id not in lane_ids:
                lane_ids.append(lane_id)
        if not lane_ids:
            return None

        geometry_cache = getattr(self, "lookahead_geometry_cache", None)
        if geometry_cache is None:
            geometry_cache = {}
            self.lookahead_geometry_cache = geometry_cache
        route_key = tuple(lane_ids)
        compiled_path = geometry_cache.get(route_key)
        if compiled_path is None:
            lane_shapes = []
            valid_lane_ids = []
            for lane_id in lane_ids:
                try:
                    lane_shape = self._get_lookahead_lane_shape(
                        lane_id, profile_ctx=profile_ctx
                    )
                except Exception:
                    continue
                if lane_shape is not None:
                    lane_shapes.append(lane_shape)
                    valid_lane_ids.append(lane_id)
            if lane_shapes:
                compiled_path = self._profile_detail_python_call(
                    profile_ctx,
                    "compile_lane_shapes",
                    compile_lane_shapes,
                    lane_shapes,
                )
                if compiled_path is not None:
                    geometry_cache[route_key] = compiled_path
                    valid_route_key = tuple(valid_lane_ids)
                    geometry_cache.setdefault(valid_route_key, compiled_path)

        return compiled_path

    def _populate_vehicle_lookaheads(self, requests, profile_ctx=None):
        if not requests:
            return

        compiled_paths = []
        current_positions = []
        lookahead_distances = []
        z_values = []
        for vehicle_id, vehicle_state, context_values in requests:
            compiled_paths.append(
                self._get_vehicle_lookahead_compiled_path(
                    vehicle_id,
                    context_values=context_values,
                    profile_ctx=profile_ctx,
                )
            )
            feedback_positions = getattr(self, "feedback_observed_positions", {})
            current_position = feedback_positions.get(
                vehicle_id, (vehicle_state.x, vehicle_state.y)
            )
            current_positions.append(current_position)
            vehicle_state.lookahead_origin_x = current_position[0]
            vehicle_state.lookahead_origin_y = current_position[1]
            lookahead_distances.append(self._lookahead_distance(vehicle_state.speed))
            z_values.append(vehicle_state.z)

        effective_distances, heading_changes = self._profile_detail_python_call(
            profile_ctx,
            "adapt_lookahead_distances_batch",
            adapt_lookahead_distances_for_compiled_paths,
            compiled_paths,
            current_positions,
            lookahead_distances,
            getattr(self, "lookahead_curve_min_distance", 3.5),
            getattr(self, "lookahead_curve_start_radians", math.radians(5.0)),
            getattr(self, "lookahead_curve_full_scale_radians", math.radians(45.0)),
        )
        lookaheads = self._profile_detail_python_call(
            profile_ctx,
            "find_lookahead_positions_batch",
            find_lookahead_positions_from_compiled_paths,
            compiled_paths,
            current_positions,
            effective_distances,
            z_values,
        )
        for request, lookahead, lookahead_distance, heading_change in zip(
            requests, lookaheads, effective_distances, heading_changes
        ):
            _, vehicle_state, context_values = request
            vehicle_state.lookahead_distance = lookahead_distance
            vehicle_state.lookahead_heading_change = heading_change
            has_lateral_speed, lateral_speed = self._context_vehicle_value(
                context_values, "VAR_SPEED_LAT"
            )
            if not has_lateral_speed:
                lateral_speed = 0.0
            has_lateral_offset, lateral_offset = self._context_vehicle_value(
                context_values, "VAR_LANEPOSITION_LAT"
            )
            if not has_lateral_offset:
                lateral_offset = getattr(vehicle_state, "lateral_offset", 0.0)
            vehicle_state.lateral_speed = lateral_speed
            lookahead, lane_change_blend = self._profile_detail_python_call(
                profile_ctx,
                "blend_lane_change_lookahead",
                blend_lane_change_lookahead,
                lookahead,
                (vehicle_state.lookahead_origin_x, vehicle_state.lookahead_origin_y),
                vehicle_state.sumo_angle,
                lookahead_distance,
                lateral_speed,
                lateral_offset,
                vehicle_state.z,
                getattr(self, "lookahead_lane_change_speed_start", 0.05),
                getattr(self, "lookahead_lane_change_speed_full", 0.35),
                getattr(self, "lookahead_lane_change_offset_start", 0.15),
                getattr(self, "lookahead_lane_change_offset_full", 0.75),
            )
            vehicle_state.lookahead_lane_change_blend = lane_change_blend
            if lookahead is None:
                continue
            vehicle_state.lookahead_x = lookahead[0]
            vehicle_state.lookahead_y = lookahead[1]
            vehicle_state.lookahead_z = lookahead[2]
            vehicle_state.lookahead_position_valid = True

    def _populate_vehicle_lookahead(
        self,
        vehicle_id,
        vehicle_state,
        profile_ctx=None,
        context_values=None,
    ):
        self._populate_vehicle_lookaheads(
            [(vehicle_id, vehicle_state, context_values or {})],
            profile_ctx=profile_ctx,
        )

    def _get_traffic_light_information(self, traffic_light_id):
        """Return cached JSON for immutable SUMO network signal metadata."""
        information = self.traffic_light_information_cache.get(traffic_light_id)
        if information is not None:
            return information
        tls_information = {"programs": {}}
        tls = self.simulator.sumo_net.getTLS(traffic_light_id)
        for program_id, program in tls.getPrograms().items():
            tls_information["programs"][program_id] = {
                "parameters": program.getParams()
            }
        information = json.dumps(tls_information, separators=(",", ":"))
        self.traffic_light_information_cache[traffic_light_id] = information
        return information

    def _build_simulation_state(self, simulator):
        """Collect the current simulation state from SUMO into a SimulationState.

        Pure state construction (no Redis/network I/O); raises on TraCI errors.
        Shared by the Redis-backed `_write_simulation_state` and the direct
        (gRPC) plugin, which returns the result over its Tick/GetState RPC
        instead of writing to Redis.
        """
        simulation_state = SimulationState()
        simulation_state.simulation_time = traci.simulation.getTime()
        profile_ctx = getattr(simulator, "ctx", None)

        # Get all interested agent IDs
        id_selection_start = time.perf_counter()
        vehicle_ids, vru_ids, construction_ids = self.get_vehicle_vru_ids()
        vehicle_ids, vehicle_position_cache = self._filter_vehicle_ids_for_state(
            vehicle_ids
        )
        active_vehicle_ids = set(vehicle_ids)
        lane_change_active_actor_ids = getattr(
            self, "feedback_lane_change_active_actor_ids", set()
        )
        lane_change_active_actor_ids.intersection_update(active_vehicle_ids)
        self.feedback_lane_change_active_actor_ids = lane_change_active_actor_ids
        detail_vehicle_ids = self._update_state_detail_active_vehicle_ids(
            vehicle_ids, vehicle_position_cache
        )
        set_value(
            profile_ctx,
            "terasim_internal.state_export.detail_vehicle_count",
            len(detail_vehicle_ids),
        )
        for feedback_cache in (
            self.feedback_observed_speeds,
            self.feedback_observed_positions,
            self.feedback_source_carla_frames,
            self.lookahead_vehicle_route_cache,
        ):
            for vid in list(feedback_cache):
                if vid not in active_vehicle_ids:
                    feedback_cache.pop(vid, None)
        simulation_state.agent_count = {
            "vehicle": len(vehicle_ids),
            "vru": len(vru_ids),
            "construction": len(construction_ids),
        }
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.id_selection_filter_s",
            time.perf_counter() - id_selection_start,
        )

        # Basic pose/shape state is exported for every CARLA actor. Ackermann-only
        # state (lookahead, desired speed, feedback acknowledgement) is limited
        # to the same AV-centred radius used for CARLA physics.
        vehicle_collection_start = time.perf_counter()
        vehicles = {}
        lookahead_requests = []
        context_results = getattr(self, "state_vehicle_context_results", {})
        for vid in vehicle_ids:
            vehicle_state = AgentStateSimplified()
            context_values = context_results.get(vid, {})
            position = vehicle_position_cache.get(vid)
            if position is None:
                position = context_values.get(traci.constants.VAR_POSITION3D)
            if position is None:
                position = traci.vehicle.getPosition3D(vid)
            vehicle_state.x, vehicle_state.y, vehicle_state.z = position
            sumo_angle = context_values.get(traci.constants.VAR_ANGLE)
            if sumo_angle is None:
                sumo_angle = traci.vehicle.getAngle(vid)
            vehicle_state.sumo_angle = sumo_angle
            vehicle_state.orientation = math.radians(
                (90 - vehicle_state.sumo_angle) % 360
            )
            speed = context_values.get(traci.constants.VAR_SPEED)
            if speed is None:
                speed = traci.vehicle.getSpeed(vid)
            vehicle_state.speed = speed
            lane_id = context_values.get(traci.constants.VAR_LANE_ID)
            if lane_id is not None:
                vehicle_state.lane_id = lane_id
            lane_position = context_values.get(traci.constants.VAR_LANEPOSITION)
            if lane_position is not None:
                vehicle_state.lane_position = lane_position
            lateral_offset = context_values.get(
                traci.constants.VAR_LANEPOSITION_LAT
            )
            if lateral_offset is not None:
                vehicle_state.lateral_offset = lateral_offset
            lateral_speed = context_values.get(traci.constants.VAR_SPEED_LAT)
            if lateral_speed is not None:
                vehicle_state.lateral_speed = lateral_speed
            lane_change_active = (
                abs(vehicle_state.lateral_speed)
                >= getattr(self, "lookahead_lane_change_speed_start", 0.05)
                or abs(vehicle_state.lateral_offset)
                >= getattr(self, "lookahead_lane_change_offset_start", 0.15)
            )
            if lane_change_active:
                self.feedback_lane_change_active_actor_ids.add(vid)
            else:
                self.feedback_lane_change_active_actor_ids.discard(vid)
            vehicle_state.feedback_position_skipped_for_lane_change = (
                lane_change_active
            )
            self._populate_vehicle_static_state(vid, vehicle_state)

            if vid in detail_vehicle_ids:
                detail_start = time.perf_counter()
                self._populate_lane_relative_position(
                    vid,
                    vehicle_state,
                    context_values=context_values,
                    profile_ctx=profile_ctx,
                )
                if self.state_export_geo_enabled:
                    vehicle_state.lon, vehicle_state.lat = (
                        self._profile_detail_traci_call(
                            profile_ctx,
                            "simulation_convert_geo",
                            traci.simulation.convertGeo,
                            vehicle_state.x,
                            vehicle_state.y,
                        )
                    )
                if vid == "AV" or self._is_ackermann_feedback_actor(vid):
                    try:
                        vehicle_state.sumo_desired_speed = (
                            self._profile_detail_traci_call(
                                profile_ctx,
                                "vehicle_get_speed_without_traci",
                                traci.vehicle.getSpeedWithoutTraCI,
                                vid,
                            )
                        )
                    except Exception:
                        vehicle_state.sumo_desired_speed = vehicle_state.speed
                    try:
                        vehicle_state.sumo_emergency_decel = (
                            self._profile_detail_traci_call(
                                profile_ctx,
                                "vehicle_get_emergency_decel",
                                traci.vehicle.getEmergencyDecel,
                                vid,
                            )
                        )
                    except Exception:
                        vehicle_state.sumo_emergency_decel = None
                vehicle_state.feedback_observed_speed = (
                    self.feedback_observed_speeds.get(vid)
                )
                vehicle_state.feedback_source_carla_frame = (
                    self.feedback_source_carla_frames.get(vid)
                )
                lookahead_requests.append((vid, vehicle_state, context_values))
                acceleration = context_values.get(traci.constants.VAR_ACCELERATION)
                if acceleration is None:
                    acceleration = self._profile_detail_traci_call(
                        profile_ctx,
                        "vehicle_get_acceleration",
                        traci.vehicle.getAcceleration,
                        vid,
                    )
                vehicle_state.acceleration = acceleration
                now_time = simulation_state.simulation_time
                now_orientation = vehicle_state.orientation
                last_orientation, last_time = self.last_orientations.get(
                    vid, (now_orientation, now_time)
                )
                dt = now_time - last_time
                if dt > 0:
                    angle_delta = now_orientation - last_orientation
                    dtheta = math.atan2(math.sin(angle_delta), math.cos(angle_delta))
                    vehicle_state.angular_velocity = dtheta / dt
                self.last_orientations[vid] = (now_orientation, now_time)
                add_timing(
                    profile_ctx,
                    "terasim_internal.state_export.ackermann_detail_s",
                    time.perf_counter() - detail_start,
                )
            else:
                self.last_orientations.pop(vid, None)

            vehicles[vid] = vehicle_state

        lookahead_start = time.perf_counter()
        self._populate_vehicle_lookaheads(lookahead_requests, profile_ctx=profile_ctx)
        lookahead_elapsed = time.perf_counter() - lookahead_start
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.lookahead_lane_geometry_s",
            lookahead_elapsed,
        )
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.ackermann_detail_s",
            lookahead_elapsed,
        )

        simulation_state.agent_details["vehicle"] = vehicles
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.vehicle_state_collection_s",
            time.perf_counter() - vehicle_collection_start,
        )

        # Add VRU states
        vru_collection_start = time.perf_counter()
        # Reuse the type sets collected at the start of this state export.
        current_vehicle_list = self._current_vehicle_id_set
        current_person_list = self._current_person_id_set

        vrus = {}
        for vru_id in vru_ids:
            vru_state = AgentStateSimplified()

            # Determine if this VRU is actually a vehicle or person
            if vru_id in current_vehicle_list:
                # VRU is actually a vehicle (disguised as pedestrian)
                vru_state.x, vru_state.y, vru_state.z = traci.vehicle.getPosition3D(vru_id)
                vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                vru_state.sumo_angle = traci.vehicle.getAngle(vru_id)
                vru_state.speed = traci.vehicle.getSpeed(vru_id)
                vru_state.acceleration = traci.vehicle.getAcceleration(vru_id)
                vru_state.length = traci.vehicle.getLength(vru_id)
                vru_state.width = traci.vehicle.getWidth(vru_id)
                vru_state.height = traci.vehicle.getHeight(vru_id)
                vru_state.type = traci.vehicle.getTypeID(vru_id)
                vru_state.angular_velocity = 0.0  # rad/s
                now_time = simulation_state.simulation_time
                now_orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                last_orientation, last_time = self.last_orientations.get(vru_id, (now_orientation, now_time))
                dt = now_time - last_time
                if dt > 0:
                    dtheta = np.arctan2(np.sin(now_orientation - last_orientation), np.cos(now_orientation - last_orientation))
                    vru_state.angular_velocity = dtheta / dt
                else:
                    vru_state.angular_velocity = 0.0
                self.last_orientations[vru_id] = (now_orientation, now_time)
                vru_state.orientation = now_orientation
            elif vru_id in current_person_list:
                # VRU is actually a person
                vru_state.x, vru_state.y, vru_state.z = traci.person.getPosition3D(vru_id)
                vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                vru_state.sumo_angle = traci.person.getAngle(vru_id)
                vru_state.speed = traci.person.getSpeed(vru_id)
                vru_state.acceleration = traci.person.getAcceleration(vru_id) if hasattr(traci.person, 'getAcceleration') else 0.0
                vru_state.length = traci.person.getLength(vru_id)
                vru_state.width = traci.person.getWidth(vru_id)
                vru_state.height = traci.person.getHeight(vru_id)
                vru_state.type = traci.person.getTypeID(vru_id)
                vru_state.angular_velocity = 0.0  # rad/s
                vru_state.orientation = np.radians((90 - vru_state.sumo_angle) % 360)
            else:
                # VRU ID not found in either list, log warning and skip
                self.logger.warning(f"VRU ID {vru_id} not found in vehicle or person lists, skipping")
                continue

            vrus[vru_id] = vru_state

        simulation_state.agent_details["vru"] = vrus
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.vru_state_collection_s",
            time.perf_counter() - vru_collection_start,
        )

        # Add construction objects
        construction_collection_start = time.perf_counter()
        construction_objects = {}
        for cid in construction_ids:
            construction_state = AgentStateSimplified()
            construction_state.x, construction_state.y, construction_state.z = traci.vehicle.getPosition3D(cid)
            construction_state.lon, construction_state.lat = traci.simulation.convertGeo(construction_state.x, construction_state.y)
            construction_state.sumo_angle = traci.vehicle.getAngle(cid)
            construction_state.orientation = np.radians((90 - construction_state.sumo_angle) % 360)
            construction_state.speed = traci.vehicle.getSpeed(cid)
            construction_state.acceleration = traci.vehicle.getAcceleration(cid)
            construction_state.length = traci.vehicle.getLength(cid)
            construction_state.width = traci.vehicle.getWidth(cid)
            construction_state.height = traci.vehicle.getHeight(cid)
            construction_state.type = traci.vehicle.getTypeID(cid)
            construction_state.angular_velocity = 0.0
            construction_objects[cid] = construction_state

        simulation_state.construction_objects = construction_objects
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.construction_state_collection_s",
            time.perf_counter() - construction_collection_start,
        )

        # Add dynamic signal phases while reusing immutable program metadata.
        traffic_light_export_start = time.perf_counter()
        traffic_lights = {}
        if self.traffic_light_id_cache is None:
            self.traffic_light_id_cache = tuple(traci.trafficlight.getIDList())
        for tl_id in self.traffic_light_id_cache:
            sumo_signal = SUMOSignal()
            sumo_signal.x, sumo_signal.y = 0, 0
            sumo_signal.tls = traci.trafficlight.getRedYellowGreenState(tl_id)
            sumo_signal.information = self._get_traffic_light_information(tl_id)
            traffic_lights[tl_id] = sumo_signal

        simulation_state.traffic_light_details = traffic_lights
        add_timing(
            profile_ctx,
            "terasim_internal.state_export.traffic_light_state_export_s",
            time.perf_counter() - traffic_light_export_start,
        )

        # Add construction zone shapes
        if self.construction_zone_shapes is None and simulator.env.static_adversity is not None and simulator.env.static_adversity.adversities is not None:
            self.construction_zone_shapes = {}
            for adversity in simulator.env.static_adversity.adversities:
                if isinstance(adversity, ConstructionAdversity):
                    lane_shape = traci.lane.getShape(adversity._lane_id)
                    if lane_shape: # convert to list of lists
                        lane_shape = interpolate_by_distance(lane_shape, 2.0)
                        lane_index = int(adversity._lane_id.split("_")[-1])
                        edge_id = traci.lane.getEdgeID(adversity._lane_id)
                        if lane_index == 0:
                            # From right to left
                            direction = 1
                        elif lane_index == traci.edge.getLaneNumber(edge_id) - 1:
                            # From left to right
                            direction = -1
                        else:
                            # Middle lane, no construction zone
                            continue
                        construction_zone_shape = generate_construction_zone_shape(lane_shape, traci.lane.getWidth(adversity._lane_id), direction)
                        self.construction_zone_shapes[adversity._lane_id] = construction_zone_shape

        simulation_state.construction_zone_details = self.construction_zone_shapes
        return simulation_state

    def _write_simulation_state(self, simulator):
        """Write the current simulation state to Redis.

        Args:
            simulator (Simulator): The simulator object.
        """
        if not self._check_simulation_status():
            return False
        try:
            simulation_state = self._build_simulation_state(simulator)
            
            # Write to Redis with expiration
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:state", simulation_state.model_dump_json()
            )
            self.redis_client.expire(
                f"simulation:{self.simulation_uuid}:state", self.key_expiry
            )
            
            # If we reach here, TeraSim is working normally
            self.error_count = 0
            self.last_successful_operation = time.time()
            return True

        except Exception as e:
            self.error_count += 1
            error_msg = str(e).lower()
            
            # Check if this is a critical error
            critical_errors = [
                "no network loaded",
                "connection lost", 
                "traci",
                "sumo",
                "simulation crashed"
            ]
            
            is_critical = any(err in error_msg for err in critical_errors)
            
            self.logger.error(f"TeraSim error #{self.error_count}: {e}")
            
            # Stop if critical error or too many consecutive errors
            if is_critical or self.error_count >= 3:
                self.logger.critical(f"TeraSim appears broken, stopping simulation")
                # Set error flag for cleanup task to handle
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop", 
                    f"terasim_error_{self.error_count}",
                    ex=300  # 5 minutes expiry
                )
                return False
                
            # Also stop if no successful operation for too long
            if time.time() - self.last_successful_operation > 300:  # 5 minutes
                self.logger.critical("TeraSim not responding for 5 minutes, stopping")
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop", 
                    "terasim_timeout",
                    ex=300
                )
                return False
                
            return True

    def _handle_agent_command(self, command_data):
        """Validate a wire command and delegate to the shared structured path."""
        parse_start = time.perf_counter()
        try:
            if isinstance(command_data, AgentCommand):
                command = command_data
            elif isinstance(command_data, dict):
                command = AgentCommand.model_validate(command_data)
            else:
                if isinstance(command_data, bytes):
                    command_data = command_data.decode("utf-8")
                command = AgentCommand.model_validate_json(command_data)
        except Exception as exc:
            self.last_agent_command_failure = {
                "reason": "agent_command_parse_error",
                "exception_type": type(exc).__name__,
            }
            self.logger.error(f"Error parsing agent command: {exc}")
            return False
        return self._apply_agent_command(
            command, parse_elapsed=time.perf_counter() - parse_start
        )

    def _apply_agent_command(self, command, parse_elapsed=0.0):
        """Apply an already validated command; shared by all transports."""
        self.last_agent_command_failure = None
        profile_ctx = getattr(self, "_active_agent_command_profile_ctx", None)
        feedback_command_start = time.perf_counter()
        is_ackermann_feedback = False
        try:
            if command.agent_id != '':
                if command.agent_type not in ["vehicle", "vru"]:
                    self.logger.error(f"Invalid agent type: {command.agent_type}")
                    return False
                if command.agent_id in self.controlled_agents_each_step:
                    self.logger.debug(f"Agent {command.agent_id} is already controlled")
                    return True
                self.controlled_agents_each_step.add(command.agent_id)
                if command.command_type == "set_state":
                    # Check that exactly one of position or lonlat is present
                    has_position = "position" in command.data
                    has_lonlat = "lonlat" in command.data
                    if not (has_position ^ has_lonlat):  # XOR operation ensures exactly one is True
                        self.logger.error("Must specify exactly one of position or lonlat")
                        return False
                    if "position" in command.data:
                        x, y = command.data["position"]
                    elif "lonlat" in command.data:
                        lon, lat = command.data["lonlat"]
                        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
                    if command.agent_type == "vehicle":
                        is_ackermann_feedback = "source_carla_frame" in command.data
                        if is_ackermann_feedback:
                            add_timing(
                                profile_ctx,
                                "terasim_internal.feedback_command_breakdown.parse_s",
                                parse_elapsed,
                            )
                            self._profile_feedback_command_call(
                                profile_ctx,
                                "python",
                                "ensure_lane_change_settings",
                                self._ensure_ackermann_feedback_lane_change_settings,
                                command.agent_id,
                            )
                        use_move_to = is_ackermann_feedback and getattr(
                            self, "ackermann_feedback_position_mode", "moveTo"
                        ) == "moveTo"
                        lane_index = (
                            getattr(self, "ackermann_feedback_lane_index", 0)
                            if is_ackermann_feedback
                            else 0
                        )
                        keep_route = (
                            getattr(self, "ackermann_feedback_keep_route", 0)
                            if is_ackermann_feedback
                            else 0
                        )
                        log_lane_transition = is_ackermann_feedback and getattr(
                            self,
                            "ackermann_feedback_log_lane_transitions",
                            False,
                        )
                        before_lane_state = None
                        if log_lane_transition:
                            before_lane_state = self._read_ackermann_feedback_lane_state(
                                command.agent_id
                            )
                            previous_lane_state = getattr(
                                self,
                                "feedback_lane_states",
                                {},
                            ).get(command.agent_id)
                            if previous_lane_state is None and before_lane_state is not None:
                                self.logger.info(
                                    "Ackermann feedback lane initial actor=%s road=%s "
                                    "lane=%s lanePos=%s routeIndex=%s",
                                    command.agent_id,
                                    before_lane_state["road_id"],
                                    before_lane_state["lane_id"],
                                    before_lane_state["lane_position"],
                                    before_lane_state["route_index"],
                                )
                            else:
                                self._log_ackermann_feedback_lane_transition(
                                    command.agent_id,
                                    "sumo_step",
                                    previous_lane_state,
                                    before_lane_state,
                                )
                        lane_change_active = command.agent_id in getattr(
                            self, "feedback_lane_change_active_actor_ids", set()
                        )
                        position_feedback_start = time.perf_counter()
                        if use_move_to and lane_change_active:
                            projection = None
                            feedback_move_source = (
                                "feedback_moveTo_lane_change_skipped"
                            )
                            add_timing(
                                profile_ctx,
                                "terasim_internal.feedback_command_breakdown.lane_change_skips",
                                1.0,
                            )
                        elif use_move_to:
                            projection = self._move_ackermann_feedback_actor(
                                command.agent_id,
                                (x, y),
                                command.data.get("sumo_angle", 0),
                                profile_ctx,
                            )
                            if projection is None:
                                if not self._should_continue_after_agent_command_failure():
                                    return False
                                # Keep SUMO longitudinal position for this step, but
                                # still apply the observed CARLA speed below.
                                feedback_move_source = "feedback_moveTo_skipped"
                            else:
                                feedback_move_source = "feedback_moveTo"
                        else:
                            # keepRoute=0 snaps externally-driven vehicles to a network
                            # lane. This is retained for non-feedback commands and the
                            # explicitly selected legacy moveToXY feedback mode.
                            traci.vehicle.moveToXY(
                                command.agent_id,
                                "",
                                lane_index,
                                x,
                                y,
                                command.data.get("sumo_angle", 0),
                                keep_route,
                            )
                            feedback_move_source = "feedback_moveToXY"
                        if is_ackermann_feedback:
                            add_timing(
                                profile_ctx,
                                "terasim_internal.feedback_command_breakdown.position_feedback_s",
                                time.perf_counter() - position_feedback_start,
                            )
                        if log_lane_transition:
                            after_lane_state = self._read_ackermann_feedback_lane_state(
                                command.agent_id
                            )
                            self._log_ackermann_feedback_lane_transition(
                                command.agent_id,
                                feedback_move_source,
                                before_lane_state,
                                after_lane_state,
                            )
                            if after_lane_state is not None:
                                if not hasattr(self, "feedback_lane_states"):
                                    self.feedback_lane_states = {}
                                self.feedback_lane_states[command.agent_id] = after_lane_state

                        # 3-cosim fix (dense maps, e.g. Odaiba): right after moveToXY, append one
                        # successor edge so the externally-driven AV's route is never a single
                        # terminal edge. With keepRoute=0 a dense network can map the AV onto an
                        # off-route edge, collapsing its route to that one edge; the AV then reaches
                        # that edge's end, SUMO retires it as "arrived", NADE stops with
                        # finish_reason "AV_left", and the cosim crashes. The AV's pose is driven
                        # entirely by moveToXY (it mirrors the Autoware ego), so this 2-edge route is
                        # only a decoy to keep it alive -- NOT a fixed plan, which is correct because
                        # the Autoware ego chooses its path dynamically.
                        if (
                            not use_move_to
                            and keep_route == 0
                            and command.agent_id == "AV"
                            and "AV" in traci.vehicle.getIDList()
                        ):
                            try:
                                cur = traci.vehicle.getRoadID("AV")
                                if cur and not cur.startswith(":"):  # skip junction-internal edges
                                    nxt = ""
                                    for lk in traci.lane.getLinks(traci.vehicle.getLaneID("AV")):
                                        if lk and lk[0]:
                                            e = traci.lane.getEdgeID(lk[0])
                                            if e and not e.startswith(":"):
                                                nxt = e
                                                break
                                    if nxt:
                                        traci.vehicle.setRoute("AV", [cur, nxt])
                            except Exception:
                                pass

                        if "speed" in command.data:
                            if is_ackermann_feedback:
                                self._profile_feedback_command_call(
                                    profile_ctx,
                                    "traci",
                                    "vehicle_set_previous_speed",
                                    traci.vehicle.setPreviousSpeed,
                                    command.agent_id,
                                    command.data["speed"],
                                )
                            else:
                                traci.vehicle.setPreviousSpeed(
                                    command.agent_id, command.data["speed"]
                                )
                            if "source_carla_frame" in command.data:
                                self.feedback_observed_speeds[command.agent_id] = command.data[
                                    "speed"
                                ]
                                if not hasattr(self, "feedback_observed_positions"):
                                    self.feedback_observed_positions = {}
                                self.feedback_observed_positions[command.agent_id] = (x, y)
                                self.feedback_source_carla_frames[command.agent_id] = command.data[
                                    "source_carla_frame"
                                ]
                    else:  # VRU type
                        # Check if VRU is actually a vehicle or person
                        current_vehicle_list = traci.vehicle.getIDList()
                        current_person_list = traci.person.getIDList()
                        
                        if command.agent_id in current_vehicle_list:
                            # VRU is actually a vehicle (disguised as pedestrian)
                            traci.vehicle.moveToXY(
                                command.agent_id, "", 0, x, y, command.data.get("sumo_angle", 0), 2
                            )
                            if "speed" in command.data:
                                traci.vehicle.setPreviousSpeed(command.agent_id, command.data["speed"])
                        elif command.agent_id in current_person_list:
                            # VRU is actually a person
                            traci.person.moveToXY(
                                command.agent_id, "", x, y, command.data.get("sumo_angle", 0), 2
                            )
                            if "speed" in command.data:
                                traci.person.setSpeed(command.agent_id, command.data["speed"])
                        else:
                            self.logger.error(f"VRU ID {command.agent_id} not found in vehicle or person lists")
                            return False
            

                self.logger.debug(f"Agent command executed: {command}")
                return True

        except Exception as e:
            if self.last_agent_command_failure is None:
                failed_command = locals().get("command")
                self.last_agent_command_failure = {
                    "reason": "agent_command_exception",
                    "exception_type": type(e).__name__,
                    "actor_id": getattr(failed_command, "agent_id", ""),
                    "ackermann_feedback": bool(
                        getattr(failed_command, "data", {}).get("source_carla_frame")
                    ),
                }
            self.logger.error(f"Error handling agent command: {e}")
            return False
        finally:
            if is_ackermann_feedback:
                add_timing(
                    profile_ctx,
                    "terasim_internal.feedback_command_breakdown.total_s",
                    time.perf_counter() - feedback_command_start,
                )
                add_timing(
                    profile_ctx,
                    "terasim_internal.feedback_command_breakdown.command_count",
                    1.0,
                )

    def _record_agent_command_failure(self, simulator):
        failure = self.last_agent_command_failure or {
            "reason": "agent_command_rejected",
        }
        self.logger.critical(
            "Fail-closed: refusing SUMO simulationStep after agent command failure: %s",
            failure,
        )
        try:
            simulator.env.record["finish_reason"] = failure["reason"]
            simulator.env.record["failed_actor"] = failure.get("actor_id", "")
        except Exception:
            pass
        simulator.running = False
        try:
            if self.redis_client:
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:status",
                    "error",
                    ex=self.key_expiry,
                )
        except Exception as exc:
            self.logger.error("Failed to publish fail-closed status: %s", exc)
        return failure

    def _reconnect_redis(self):
        """Reconnect to Redis server.

        Returns:
            bool: True if reconnection was successful, False otherwise.
        """
        try:
            self.logger.info("Attempting to reconnect to Redis...")
            self.redis_client = redis.Redis(**self.redis_config)
            self.logger.info("Successfully reconnected to Redis")
            return True
        except RedisError as e:
            self.logger.error(f"Failed to reconnect to Redis: {e}")
            return False

    def _handle_pending_agent_commands(self, simulator=None):
        """Handle all pending agent commands in the queue."""
        if not self._check_simulation_status():
            return False
        try:
            # The cap prevents an infinite producer loop while allowing one feedback batch.
            for _ in range(self.max_agent_commands_per_step):
                command_data = self.redis_client.lpop(
                    f"simulation:{self.simulation_uuid}:agent_commands"
                )
                if not command_data:
                    break

                if not self._handle_agent_command(command_data):
                    if self._should_continue_after_agent_command_failure():
                        continue
                    if simulator is not None:
                        self._record_agent_command_failure(simulator)
                    return False
            return True
        except Exception as e:
            self.logger.error(f"Error handling pending agent commands: {e}")
            self.last_agent_command_failure = {
                "reason": "pending_agent_command_exception",
                "exception_type": type(e).__name__,
            }
            if simulator is not None:
                self._record_agent_command_failure(simulator)
            return False

    def _extract_map_geometry(self, sumo_net):
        """Extract static map geometry from SUMO network."""
        map_data = {
            "lanes": [],
            "edges": [],  # Add edges for boundary calculation
            "junctions": [],
            "traffic_lights": [],
            "bounds": {
                "min_x": float('inf'),
                "max_x": float('-inf'),
                "min_y": float('inf'),
                "max_y": float('-inf')
            }
        }
        
        # Extract lane data
        for edge in sumo_net.getEdges():
            for lane in edge.getLanes():
                lane_shape = lane.getShape()
                if lane_shape:
                    # Convert to list of lists and update bounds
                    shape_list = []
                    for x, y in lane_shape:
                        shape_list.append([float(x), float(y)])
                        map_data["bounds"]["min_x"] = min(map_data["bounds"]["min_x"], x)
                        map_data["bounds"]["max_x"] = max(map_data["bounds"]["max_x"], x)
                        map_data["bounds"]["min_y"] = min(map_data["bounds"]["min_y"], y)
                        map_data["bounds"]["max_y"] = max(map_data["bounds"]["max_y"], y)
                    
                    map_data["lanes"].append({
                        "id": lane.getID(),
                        "shape": shape_list,
                        "width": float(lane.getWidth()),
                        "speed_limit": float(lane.getSpeed()),
                        "length": float(lane.getLength()),
                        "edge_id": edge.getID()
                    })
        
        # Calculate all lane boundaries for each edge
        edges_dict = {}
        for lane_data in map_data["lanes"]:
            edge_id = lane_data["edge_id"]
            if edge_id not in edges_dict:
                edges_dict[edge_id] = []
            edges_dict[edge_id].append(lane_data)
        
        # For each edge, calculate all lane boundaries
        for edge_id, lanes in edges_dict.items():
            if not lanes:
                continue
            
            # Sort lanes by their index (assuming lane IDs end with _0, _1, etc.)
            lanes.sort(key=lambda l: int(l["id"].split("_")[-1]))
            
            # Calculate boundary for each lane
            lane_boundaries = []
            
            # Helper function to calculate boundary line
            def calculate_boundary(lane_shape, lane_width, side):
                """Calculate left or right boundary of a lane.
                side: -1 for left boundary, 1 for right boundary
                """
                boundary = []
                for i, point in enumerate(lane_shape):
                    # Calculate perpendicular vector
                    if i < len(lane_shape) - 1:
                        # Use next point for direction
                        dx = lane_shape[i+1][0] - point[0]
                        dy = lane_shape[i+1][1] - point[1]
                    else:
                        # For last point, use previous point for direction
                        dx = point[0] - lane_shape[i-1][0]
                        dy = point[1] - lane_shape[i-1][1]
                    
                    length = (dx**2 + dy**2)**0.5
                    if length > 0:
                        # Perpendicular vector (left: -dy,dx; right: dy,-dx)
                        if side < 0:  # left
                            perp_x = -dy / length
                            perp_y = dx / length
                        else:  # right
                            perp_x = dy / length
                            perp_y = -dx / length
                        
                        # Offset by half lane width
                        offset = lane_width / 2
                        boundary.append([
                            point[0] + perp_x * offset,
                            point[1] + perp_y * offset
                        ])
                return boundary
            
            # Calculate boundaries for all lanes
            all_boundaries = []
            
            # Right boundary of the rightmost lane (road edge)
            if lanes:
                right_edge = calculate_boundary(lanes[0]["shape"], lanes[0]["width"], 1)
                if right_edge:
                    all_boundaries.append({
                        "points": right_edge,
                        "type": "edge",
                        "side": "right"
                    })
            
            # Boundaries between lanes
            for i in range(len(lanes) - 1):
                # Left boundary of current lane = right boundary of next lane
                lane_divider = calculate_boundary(lanes[i]["shape"], lanes[i]["width"], -1)
                if lane_divider:
                    all_boundaries.append({
                        "points": lane_divider,
                        "type": "divider",
                        "between": [lanes[i]["id"], lanes[i+1]["id"]]
                    })
            
            # Left boundary of the leftmost lane (road edge)
            if lanes:
                left_edge = calculate_boundary(lanes[-1]["shape"], lanes[-1]["width"], -1)
                if left_edge:
                    all_boundaries.append({
                        "points": left_edge,
                        "type": "edge",
                        "side": "left"
                    })
            
            if all_boundaries:
                map_data["edges"].append({
                    "id": edge_id,
                    "boundaries": all_boundaries,
                    "lanes": [l["id"] for l in lanes]
                })
        
        # Extract junction data
        for node in sumo_net.getNodes():
            if node.getType() not in ["dead_end", "rail_crossing"]:
                shape = node.getShape()
                if shape:
                    shape_list = [[float(x), float(y)] for x, y in shape]
                    coord = node.getCoord()
                    map_data["junctions"].append({
                        "id": node.getID(),
                        "shape": shape_list,
                        "position": [float(coord[0]), float(coord[1])],
                        "type": node.getType()
                    })
        
        # Extract traffic light data with actual positions
        # TODO: Fix TLS API - getNodes() doesn't exist, need to find correct method
        # Temporarily commented out to allow visualization to start
        '''
        for tls in sumo_net.getTrafficLights():
            tls_id = tls.getID()
            
            # Get controlled nodes to find traffic light positions
            controlled_nodes = tls.getNodes()
            
            for node in controlled_nodes:
                coord = node.getCoord()
                
                # Get controlled lanes for this traffic light
                controlled_lanes = []
                for connection in tls.getConnections():
                    from_lane_id = connection.getFromLane().getID()
                    to_lane_id = connection.getToLane().getID()
                    controlled_lanes.append({
                        "from": from_lane_id,
                        "to": to_lane_id
                    })
                
                map_data["traffic_lights"].append({
                    "id": tls_id,
                    "position": [float(coord[0]), float(coord[1])],
                    "node_id": node.getID(),
                    "controlled_lanes": controlled_lanes
                })
        '''
        
        return map_data

    def _start_streamlit_service(self):
        """Start the Dash visualization service."""
        try:
            # Path to Dash app (now using Dash by default)
            viz_app_path = Path(__file__).parent / "dash_viz_app.py"
            
            if not viz_app_path.exists():
                self.logger.error(f"Dash app not found at {viz_app_path}")
                return
            
            # Start Dash process
            cmd = [
                "python", str(viz_app_path),
                "--simulation_uuid", self.simulation_uuid,
                "--redis_host", self.redis_config.get("host", "localhost"),
                "--redis_port", str(self.redis_config.get("port", 6379)),
                "--port", str(self.viz_port),
                "--update_interval", str(1.0 / self.viz_update_freq)  # Convert frequency to interval
            ]
            
            self.viz_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.logger.info(
                f"🎨 Dash visualization started at http://localhost:{self.viz_port}"
            )
            
            # Give it a moment to start
            time.sleep(2)
            
            # Check if process started successfully
            if self.viz_process.poll() is not None:
                stdout, stderr = self.viz_process.communicate()
                self.logger.error(f"Dash process failed to start: {stderr}")
                raise RuntimeError(f"Dash process failed to start: {stderr}")
                
        except Exception as e:
            self.logger.error(f"Failed to start visualization service: {e}")


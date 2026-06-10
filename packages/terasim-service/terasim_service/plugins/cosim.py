import csv
import functools
import json
import logging
from logging.handlers import RotatingFileHandler
import numpy as np
import os
import redis
from redis.exceptions import RedisError
import time
import subprocess
from pathlib import Path

from loguru import logger as loguru_logger

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_nde_nade.adversity import ConstructionAdversity

from .base import BasePlugin, DEFAULT_REDIS_CONFIG

from ..utils import SimulationState, AgentStateSimplified, SUMOSignal, AgentCommand
from ..utils.sumo_lane_geometry import reconstruct_position_from_lane_geometry


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

        # Cache construction zone shapes
        self.construction_zone_shapes = None

        # Initialize last orientations cache
        self.last_orientations = {}  # {vehicle_id: (last_orientation, last_time)}
        
        # Initialize health monitoring
        self.error_count = 0
        self.last_successful_operation = time.time()

        self.sumo_gui_track_vehicle_id = os.getenv("SUMO_GUI_TRACK_VEHICLE", "AV")
        self.sumo_gui_track_zoom = self._parse_optional_float(
            os.getenv("SUMO_GUI_TRACK_ZOOM", "900")
        )
        self.sumo_gui_tracking_active = False
        self.sumo_gui_tracking_warning_logged = False
        self.sumo_gui_tracking_missing_logged = False
        self.idle_state_write_interval = self._parse_float_env(
            "TERASIM_COSIM_IDLE_STATE_WRITE_INTERVAL", 0.5
        )
        self._last_idle_state_write_wall_time = 0.0

        self.state_filter_enabled = self._parse_bool_env("TERASIM_COSIM_STATE_FILTER", False)
        self.state_filter_center_id = os.getenv("TERASIM_COSIM_STATE_FILTER_CENTER_ID", "AV")
        self.state_filter_radius = self._parse_optional_float(
            os.getenv("TERASIM_COSIM_STATE_FILTER_RADIUS", "")
        )
        self.state_filter_missing_center_logged = False
        self.state_filter_error_logged = False
        if self.state_filter_enabled:
            self.logger.info(
                "TeraSim co-sim state filter enabled: center=%s radius=%s",
                self.state_filter_center_id,
                self.state_filter_radius,
            )

        self.lane_relative_position_enabled = self._parse_bool_env(
            "TERASIM_COSIM_LANE_RELATIVE_POSITION", False
        )
        if self.lane_relative_position_enabled:
            self.logger.info(
                "TeraSim co-sim lane-relative reconstructed positions enabled "
                "for filtered state vehicles"
            )

        self.env_profile_enabled = self._parse_bool_env(
            "TERASIM_COSIM_ENV_PROFILE", False
        )
        self.log_profile_enabled = self._parse_bool_env(
            "TERASIM_COSIM_LOG_PROFILE", self.env_profile_enabled
        )
        self._env_profile_wrapped = False
        self._log_profile_wrapped = False
        self._log_profile_original_methods = {}
        self._current_env_profile = None
        if self.env_profile_enabled:
            self.logger.info("TeraSim co-sim env-step profiler enabled")

        self.step_profile_log = os.getenv("TERASIM_COSIM_STEP_PROFILE_LOG", "").strip()
        self.step_profile_print_every = self._parse_int_env(
            "TERASIM_COSIM_STEP_PROFILE_PRINT_EVERY", 20
        )
        self._step_profile_file = None
        self._step_profile_writer = None
        self._step_profile_rows = 0
        self._current_step_start_perf = None
        self._current_step_start_wall_time = None
        self._last_state_write_profile = {}
        self._step_profile_fieldnames = [
            "wall_time",
            "completed_sumo_time",
            "completed_tick_count",
            "total",
            "env_step",
            "after_total",
            "state_write",
            "state_check_status",
            "state_get_ids",
            "state_vehicle_filter",
            "state_raw_vehicle_count",
            "state_vehicle_loop",
            "state_vru_loop",
            "state_construction_loop",
            "state_tls_loop",
            "state_construction_zone",
            "state_json_dump",
            "state_redis_set",
            "state_redis_expire",
            "state_total",
            "state_vehicle_count",
            "state_vru_count",
            "state_construction_count",
            "state_tls_count",
            "state_json_bytes",
            "env_profile_env_step",
            "env_profile_sumo_step",
            "env_profile_preparation",
            "env_profile_nde_decision",
            "env_profile_get_env_observation",
            "env_profile_execute_move",
            "env_profile_nade_decision_and_control",
            "env_profile_nade_decision",
            "env_profile_nade_importance_sampling",
            "env_profile_refresh_control_commands_state",
            "env_profile_execute_control_commands",
            "env_profile_record_step_data",
            "env_profile_try_insert_emergency_vehicle",
            "env_profile_vehicle_count",
            "env_profile_vru_count",
            "loguru_total",
            "loguru_count",
            "loguru_trace",
            "loguru_trace_count",
            "loguru_debug",
            "loguru_debug_count",
            "loguru_info",
            "loguru_info_count",
            "loguru_warning",
            "loguru_warning_count",
            "loguru_error",
            "loguru_error_count",
            "loguru_critical",
            "loguru_critical_count",
            "completion_redis",
            "gui_tracking",
            "result",
        ]
        self._open_step_profile()

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

    def _open_step_profile(self):
        if not self.step_profile_log or self._step_profile_writer is not None:
            return
        profile_path = Path(self.step_profile_log)
        if profile_path.parent != Path(""):
            profile_path.parent.mkdir(parents=True, exist_ok=True)
        self._step_profile_file = open(profile_path, "w", newline="")
        self._step_profile_writer = csv.DictWriter(
            self._step_profile_file, fieldnames=self._step_profile_fieldnames
        )
        self._step_profile_writer.writeheader()
        self._step_profile_file.flush()

    def _record_step_profile(self, row):
        if self._step_profile_writer is None or self._step_profile_file is None:
            return
        self._step_profile_rows += 1
        output_row = {field: row.get(field, "") for field in self._step_profile_fieldnames}
        self._step_profile_writer.writerow(output_row)
        self._step_profile_file.flush()
        if (
            self.step_profile_print_every
            and self._step_profile_rows % self.step_profile_print_every == 0
        ):
            self.logger.info(
                "step profile rows=%s total=%.3fs env_step=%.3fs "
                "state_write=%.3fs vehicles=%s",
                self._step_profile_rows,
                float(output_row.get("total") or 0.0),
                float(output_row.get("env_step") or 0.0),
                float(output_row.get("state_write") or 0.0),
                output_row.get("state_vehicle_count", ""),
            )

    @staticmethod
    def _parse_optional_float(value):
        if value in (None, ""):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

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
        console_handler.setLevel(logging.INFO)

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

            self._update_sumo_gui_tracking(simulator)

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
            self._handle_pending_agent_commands()

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
        if self.env_profile_enabled:
            self._current_env_profile = self._new_env_profile()
        self._current_step_start_perf = time.perf_counter()
        self._current_step_start_wall_time = time.time()
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
        after_start = time.perf_counter()
        step_start = self._current_step_start_perf or after_start
        row = {
            "wall_time": self._current_step_start_wall_time or time.time(),
            "env_step": after_start - step_start,
            "result": "ok",
        }

        state_start = time.perf_counter()
        state_write_success = self._write_simulation_state(simulator)
        row["state_write"] = time.perf_counter() - state_start
        state_profile = self._last_state_write_profile or {}
        for key, value in state_profile.items():
            row[f"state_{key}"] = value
        env_profile = self._current_env_profile or {}
        for key, value in env_profile.items():
            if key.startswith("loguru_") or key in {"loguru_total", "loguru_count"}:
                row[key] = value
            else:
                row[f"env_profile_{key}"] = value
        if not state_write_success:
            row["result"] = "state_write_failed"
            row["after_total"] = time.perf_counter() - after_start
            row["total"] = time.perf_counter() - step_start
            self._record_step_profile(row)
            self._current_env_profile = None
            return False

        completion_start = time.perf_counter()
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
        row["completion_redis"] = time.perf_counter() - completion_start
        row["completed_sumo_time"] = completed_sumo_time
        row["completed_tick_count"] = completed_tick_count

        self._last_idle_state_write_wall_time = time.time()
        gui_start = time.perf_counter()
        self._update_sumo_gui_tracking(simulator)
        row["gui_tracking"] = time.perf_counter() - gui_start
        row["after_total"] = time.perf_counter() - after_start
        row["total"] = time.perf_counter() - step_start
        self._record_step_profile(row)
        self._current_env_profile = None
        self.logger.info(
            "Simulation step finished! completed_sumo_time=%s completed_tick_count=%s",
            completed_sumo_time,
            completed_tick_count,
        )
        return True

    def _update_sumo_gui_tracking(self, simulator: Simulator):
        """Keep SUMO GUI centered on the configured vehicle when GUI mode is active."""
        if not self.sumo_gui_track_vehicle_id:
            return
        if not getattr(simulator, "gui_flag", False):
            return

        try:
            if self.sumo_gui_track_vehicle_id not in traci.vehicle.getIDList():
                self.sumo_gui_tracking_active = False
                if not self.sumo_gui_tracking_missing_logged:
                    self.logger.info(
                        "Waiting for SUMO GUI tracked vehicle %s to appear",
                        self.sumo_gui_track_vehicle_id,
                    )
                    self.sumo_gui_tracking_missing_logged = True
                return

            if not self.sumo_gui_tracking_active:
                simulator.track_vehicle_gui(self.sumo_gui_track_vehicle_id)
                if self.sumo_gui_track_zoom is not None:
                    simulator.set_zoom(self.sumo_gui_track_zoom)
                self.sumo_gui_tracking_active = True
                self.sumo_gui_tracking_missing_logged = False
                self.logger.info(
                    "SUMO GUI tracking %s at zoom %s",
                    self.sumo_gui_track_vehicle_id,
                    self.sumo_gui_track_zoom,
                )
        except Exception as e:
            self.sumo_gui_tracking_active = False
            if not self.sumo_gui_tracking_warning_logged:
                self.logger.warning(f"SUMO GUI tracking failed: {e}")
                self.sumo_gui_tracking_warning_logged = True

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
        if self.env_profile_enabled:
            self._install_env_profile_wrappers(simulator)
            self._install_env_profile_pipeline_hooks(simulator)
        if self.log_profile_enabled:
            self._install_log_profile_wrappers()
        simulator.step_pipeline.hook(f"{self.plugin_name}_after_env_step", self.function_after_env_step, priority=self.plugin_priority["after_env"]["step"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_before_env_stop", self.function_before_env_stop, priority=self.plugin_priority["before_env"]["stop"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_after_env_stop", self.function_after_env_stop, priority=self.plugin_priority["after_env"]["stop"])
    
    def _new_env_profile(self):
        profile = {
            "env_step": 0.0,
            "sumo_step": 0.0,
            "preparation": 0.0,
            "nde_decision": 0.0,
            "get_env_observation": 0.0,
            "execute_move": 0.0,
            "nade_decision_and_control": 0.0,
            "nade_decision": 0.0,
            "nade_importance_sampling": 0.0,
            "refresh_control_commands_state": 0.0,
            "execute_control_commands": 0.0,
            "record_step_data": 0.0,
            "try_insert_emergency_vehicle": 0.0,
            "vehicle_count": 0,
            "vru_count": 0,
            "loguru_total": 0.0,
            "loguru_count": 0,
        }
        for level in ("trace", "debug", "info", "warning", "error", "critical"):
            profile[f"loguru_{level}"] = 0.0
            profile[f"loguru_{level}_count"] = 0
        return profile

    def _add_env_profile_value(self, key, elapsed):
        if self._current_env_profile is None:
            return
        self._current_env_profile[key] = self._current_env_profile.get(key, 0.0) + elapsed

    def _set_env_profile_count(self, key, value):
        if self._current_env_profile is None:
            return
        self._current_env_profile[key] = value

    def _profile_env_step_start(self, simulator, ctx):
        if self._current_env_profile is not None:
            self._current_env_profile["_env_step_start"] = time.perf_counter()
            self._set_env_profile_count(
                "vehicle_count", len(getattr(simulator.env, "vehicle_list", {}))
            )
            self._set_env_profile_count(
                "vru_count", len(getattr(simulator.env, "vulnerable_road_user_list", {}))
            )
        return True

    def _profile_env_step_end(self, simulator, ctx):
        if self._current_env_profile is not None:
            start = self._current_env_profile.pop("_env_step_start", None)
            if start is not None:
                self._add_env_profile_value("env_step", time.perf_counter() - start)
        return True

    def _profile_sumo_step_start(self, simulator, ctx):
        if self._current_env_profile is not None:
            self._current_env_profile["_sumo_step_start"] = time.perf_counter()
        return True

    def _profile_sumo_step_end(self, simulator, ctx):
        if self._current_env_profile is not None:
            start = self._current_env_profile.pop("_sumo_step_start", None)
            if start is not None:
                self._add_env_profile_value("sumo_step", time.perf_counter() - start)
        return True

    def _install_env_profile_pipeline_hooks(self, simulator):
        simulator.step_pipeline.hook(
            f"{self.plugin_name}_profile_env_step_start",
            self._profile_env_step_start,
            priority=-1,
        )
        simulator.step_pipeline.hook(
            f"{self.plugin_name}_profile_env_step_end",
            self._profile_env_step_end,
            priority=1,
        )
        simulator.step_pipeline.hook(
            f"{self.plugin_name}_profile_sumo_step_start",
            self._profile_sumo_step_start,
            priority=9,
        )
        simulator.step_pipeline.hook(
            f"{self.plugin_name}_profile_sumo_step_end",
            self._profile_sumo_step_end,
            priority=11,
        )

    def _install_env_profile_wrappers(self, simulator):
        if self._env_profile_wrapped or not hasattr(simulator, "env"):
            return

        method_map = {
            "preparation": "preparation",
            "NDE_decision": "nde_decision",
            "get_env_observation": "get_env_observation",
            "executeMove": "execute_move",
            "NADE_decision_and_control": "nade_decision_and_control",
            "NADE_decision": "nade_decision",
            "NADE_importance_sampling": "nade_importance_sampling",
            "refresh_control_commands_state": "refresh_control_commands_state",
            "execute_control_commands": "execute_control_commands",
            "record_step_data": "record_step_data",
            "try_insert_emergency_vehicle": "try_insert_emergency_vehicle",
        }

        env = simulator.env
        for method_name, profile_key in method_map.items():
            if not hasattr(env, method_name):
                continue
            original = getattr(env, method_name)
            if getattr(original, "_terasim_cosim_profile_wrapped", False):
                continue

            @functools.wraps(original)
            def wrapped(*args, _original=original, _profile_key=profile_key, **kwargs):
                start = time.perf_counter()
                try:
                    return _original(*args, **kwargs)
                finally:
                    self._add_env_profile_value(
                        _profile_key, time.perf_counter() - start
                    )

            wrapped._terasim_cosim_profile_wrapped = True
            setattr(env, method_name, wrapped)

        self._env_profile_wrapped = True

    def _install_log_profile_wrappers(self):
        if self._log_profile_wrapped:
            return

        for level in ("trace", "debug", "info", "warning", "error", "critical"):
            original = getattr(loguru_logger, level)
            self._log_profile_original_methods[level] = original

            @functools.wraps(original)
            def wrapped(*args, _original=original, _level=level, **kwargs):
                start = time.perf_counter()
                try:
                    return _original(*args, **kwargs)
                finally:
                    elapsed = time.perf_counter() - start
                    profile = self._current_env_profile
                    if profile is not None:
                        profile["loguru_total"] = profile.get("loguru_total", 0.0) + elapsed
                        profile["loguru_count"] = profile.get("loguru_count", 0) + 1
                        profile[f"loguru_{_level}"] = (
                            profile.get(f"loguru_{_level}", 0.0) + elapsed
                        )
                        profile[f"loguru_{_level}_count"] = (
                            profile.get(f"loguru_{_level}_count", 0) + 1
                        )

            setattr(loguru_logger, level, wrapped)

        self._log_profile_wrapped = True

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
        """Get all vehicle and VRU IDs in the simulation."""
        all_ids = list(set(traci.vehicle.getIDList() + traci.person.getIDList()))
        # Separate by type: construction objects, VRUs, and regular vehicles
        construction_ids = [id for id in all_ids if id.startswith("CONSTRUCTION_")]
        vru_ids = [id for id in all_ids if "VRU" in id and id not in construction_ids]
        vehicle_ids = [id for id in all_ids if id not in vru_ids and id not in construction_ids]
        return vehicle_ids, vru_ids, construction_ids

    def _filter_vehicle_ids_for_state(self, vehicle_ids):
        if (
            not self.state_filter_enabled
            or self.state_filter_radius is None
            or self.state_filter_radius <= 0
        ):
            return vehicle_ids, {}

        center_id = self.state_filter_center_id
        if center_id not in vehicle_ids:
            if not self.state_filter_missing_center_logged:
                self.logger.info(
                    "TeraSim co-sim state filter center %s is not in SUMO vehicles; "
                    "returning full state until it appears",
                    center_id,
                )
                self.state_filter_missing_center_logged = True
            return vehicle_ids, {}

        try:
            center_position = traci.vehicle.getPosition3D(center_id)
            radius_sq = self.state_filter_radius * self.state_filter_radius
            filtered_vehicle_ids = []
            position_cache = {}
            for vid in vehicle_ids:
                position = center_position if vid == center_id else traci.vehicle.getPosition3D(vid)
                dx = position[0] - center_position[0]
                dy = position[1] - center_position[1]
                if vid == center_id or dx * dx + dy * dy <= radius_sq:
                    filtered_vehicle_ids.append(vid)
                    position_cache[vid] = position
            self.state_filter_missing_center_logged = False
            return filtered_vehicle_ids, position_cache
        except Exception as e:
            if not self.state_filter_error_logged:
                self.logger.warning(
                    "TeraSim co-sim state filter failed; returning full state: %s", e
                )
                self.state_filter_error_logged = True
            return vehicle_ids, {}

    def _populate_lane_relative_position(self, vehicle_id, vehicle_state):
        if not self.lane_relative_position_enabled:
            return

        lane_id = traci.vehicle.getLaneID(vehicle_id)
        lane_position = traci.vehicle.getLanePosition(vehicle_id)
        lateral_offset = traci.vehicle.getLateralLanePosition(vehicle_id)
        lane_shape = traci.lane.getShape(lane_id)
        reconstructed = reconstruct_position_from_lane_geometry(
            lane_shape,
            lane_position,
            lateral_offset,
            vehicle_state.z,
        )
        if reconstructed is None:
            return

        vehicle_state.lane_id = lane_id
        vehicle_state.lane_position = lane_position
        vehicle_state.lateral_offset = lateral_offset
        (
            vehicle_state.reconstructed_x,
            vehicle_state.reconstructed_y,
            vehicle_state.reconstructed_z,
        ) = reconstructed
        vehicle_state.reconstructed_position_valid = True

    def _write_simulation_state(self, simulator):
        """Write the current simulation state to Redis.

        Args:
            simulator (Simulator): The simulator object.
        """
        profile = {
            "check_status": 0.0,
            "get_ids": 0.0,
            "vehicle_filter": 0.0,
            "vehicle_loop": 0.0,
            "vru_loop": 0.0,
            "construction_loop": 0.0,
            "tls_loop": 0.0,
            "construction_zone": 0.0,
            "json_dump": 0.0,
            "redis_set": 0.0,
            "redis_expire": 0.0,
            "total": 0.0,
            "vehicle_count": 0,
            "raw_vehicle_count": 0,
            "vru_count": 0,
            "construction_count": 0,
            "tls_count": 0,
            "json_bytes": 0,
            "result": "ok",
        }
        total_start = time.perf_counter()
        self._last_state_write_profile = profile

        stage_start = time.perf_counter()
        if not self._check_simulation_status():
            profile["check_status"] = time.perf_counter() - stage_start
            profile["result"] = "status_not_ok"
            profile["total"] = time.perf_counter() - total_start
            return False
        profile["check_status"] = time.perf_counter() - stage_start

        try:
            simulation_state = SimulationState()
            simulation_state.simulation_time = traci.simulation.getTime()

            stage_start = time.perf_counter()
            vehicle_ids, vru_ids, construction_ids = self.get_vehicle_vru_ids()
            profile["get_ids"] = time.perf_counter() - stage_start
            profile["raw_vehicle_count"] = len(vehicle_ids)

            stage_start = time.perf_counter()
            vehicle_ids, vehicle_position_cache = self._filter_vehicle_ids_for_state(vehicle_ids)
            profile["vehicle_filter"] = time.perf_counter() - stage_start
            profile["vehicle_count"] = len(vehicle_ids)
            profile["vru_count"] = len(vru_ids)
            profile["construction_count"] = len(construction_ids)
            simulation_state.agent_count = {
                "vehicle": len(vehicle_ids),
                "vru": len(vru_ids),
                "construction": len(construction_ids),
            }

            vehicles = {}
            stage_start = time.perf_counter()
            for vid in vehicle_ids:
                vehicle_state = AgentStateSimplified()
                position = vehicle_position_cache.get(vid)
                if position is None:
                    position = traci.vehicle.getPosition3D(vid)
                vehicle_state.x, vehicle_state.y, vehicle_state.z = position
                self._populate_lane_relative_position(vid, vehicle_state)
                vehicle_state.lon, vehicle_state.lat = traci.simulation.convertGeo(
                    vehicle_state.x, vehicle_state.y
                )
                vehicle_state.sumo_angle = traci.vehicle.getAngle(vid)
                vehicle_state.orientation = np.radians(
                    (90 - vehicle_state.sumo_angle) % 360
                )
                vehicle_state.speed = traci.vehicle.getSpeed(vid)
                vehicle_state.acceleration = traci.vehicle.getAcceleration(vid)
                vehicle_state.length = traci.vehicle.getLength(vid)
                vehicle_state.width = traci.vehicle.getWidth(vid)
                vehicle_state.height = traci.vehicle.getHeight(vid)
                vehicle_state.type = traci.vehicle.getTypeID(vid)
                vehicle_state.angular_velocity = 0.0
                now_time = simulation_state.simulation_time
                now_orientation = vehicle_state.orientation
                last_orientation, last_time = self.last_orientations.get(
                    vid, (now_orientation, now_time)
                )
                dt = now_time - last_time
                if dt > 0:
                    dtheta = np.arctan2(
                        np.sin(now_orientation - last_orientation),
                        np.cos(now_orientation - last_orientation),
                    )
                    vehicle_state.angular_velocity = dtheta / dt
                else:
                    vehicle_state.angular_velocity = 0.0
                self.last_orientations[vid] = (now_orientation, now_time)
                vehicles[vid] = vehicle_state
            profile["vehicle_loop"] = time.perf_counter() - stage_start
            simulation_state.agent_details["vehicle"] = vehicles

            current_vehicle_list = traci.vehicle.getIDList()
            current_person_list = traci.person.getIDList()

            vrus = {}
            stage_start = time.perf_counter()
            for vru_id in vru_ids:
                vru_state = AgentStateSimplified()
                if vru_id in current_vehicle_list:
                    vru_state.x, vru_state.y, vru_state.z = (
                        traci.vehicle.getPosition3D(vru_id)
                    )
                    vru_state.lon, vru_state.lat = traci.simulation.convertGeo(
                        vru_state.x, vru_state.y
                    )
                    vru_state.sumo_angle = traci.vehicle.getAngle(vru_id)
                    vru_state.speed = traci.vehicle.getSpeed(vru_id)
                    vru_state.acceleration = traci.vehicle.getAcceleration(vru_id)
                    vru_state.length = traci.vehicle.getLength(vru_id)
                    vru_state.width = traci.vehicle.getWidth(vru_id)
                    vru_state.height = traci.vehicle.getHeight(vru_id)
                    vru_state.type = traci.vehicle.getTypeID(vru_id)
                    vru_state.angular_velocity = 0.0
                    now_time = simulation_state.simulation_time
                    now_orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                    last_orientation, last_time = self.last_orientations.get(
                        vru_id, (now_orientation, now_time)
                    )
                    dt = now_time - last_time
                    if dt > 0:
                        dtheta = np.arctan2(
                            np.sin(now_orientation - last_orientation),
                            np.cos(now_orientation - last_orientation),
                        )
                        vru_state.angular_velocity = dtheta / dt
                    else:
                        vru_state.angular_velocity = 0.0
                    self.last_orientations[vru_id] = (now_orientation, now_time)
                    vru_state.orientation = now_orientation
                elif vru_id in current_person_list:
                    vru_state.x, vru_state.y, vru_state.z = (
                        traci.person.getPosition3D(vru_id)
                    )
                    vru_state.lon, vru_state.lat = traci.simulation.convertGeo(
                        vru_state.x, vru_state.y
                    )
                    vru_state.sumo_angle = traci.person.getAngle(vru_id)
                    vru_state.speed = traci.person.getSpeed(vru_id)
                    vru_state.acceleration = (
                        traci.person.getAcceleration(vru_id)
                        if hasattr(traci.person, "getAcceleration")
                        else 0.0
                    )
                    vru_state.length = traci.person.getLength(vru_id)
                    vru_state.width = traci.person.getWidth(vru_id)
                    vru_state.height = traci.person.getHeight(vru_id)
                    vru_state.type = traci.person.getTypeID(vru_id)
                    vru_state.angular_velocity = 0.0
                    vru_state.orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                else:
                    self.logger.warning(
                        f"VRU ID {vru_id} not found in vehicle or person lists, skipping"
                    )
                    continue
                vrus[vru_id] = vru_state
            profile["vru_loop"] = time.perf_counter() - stage_start
            simulation_state.agent_details["vru"] = vrus

            construction_objects = {}
            stage_start = time.perf_counter()
            for cid in construction_ids:
                construction_state = AgentStateSimplified()
                construction_state.x, construction_state.y, construction_state.z = (
                    traci.vehicle.getPosition3D(cid)
                )
                construction_state.lon, construction_state.lat = traci.simulation.convertGeo(
                    construction_state.x, construction_state.y
                )
                construction_state.sumo_angle = traci.vehicle.getAngle(cid)
                construction_state.orientation = np.radians(
                    (90 - construction_state.sumo_angle) % 360
                )
                construction_state.speed = traci.vehicle.getSpeed(cid)
                construction_state.acceleration = traci.vehicle.getAcceleration(cid)
                construction_state.length = traci.vehicle.getLength(cid)
                construction_state.width = traci.vehicle.getWidth(cid)
                construction_state.height = traci.vehicle.getHeight(cid)
                construction_state.type = traci.vehicle.getTypeID(cid)
                construction_state.angular_velocity = 0.0
                construction_objects[cid] = construction_state
            profile["construction_loop"] = time.perf_counter() - stage_start
            simulation_state.construction_objects = construction_objects

            traffic_lights = {}
            stage_start = time.perf_counter()
            for tl_id in traci.trafficlight.getIDList():
                sumo_signal = SUMOSignal()
                sumo_signal.x, sumo_signal.y = 0, 0
                sumo_signal.tls = traci.trafficlight.getRedYellowGreenState(tl_id)
                tls_information = {"programs": {}}
                tls = self.simulator.sumo_net.getTLS(tl_id)
                programs = tls.getPrograms()
                for program_id, program in programs.items():
                    program_parameters = program.getParams()
                    tls_information["programs"][program_id] = {
                        "parameters": program_parameters
                    }
                sumo_signal.information = json.dumps(tls_information)
                traffic_lights[tl_id] = sumo_signal
            profile["tls_loop"] = time.perf_counter() - stage_start
            profile["tls_count"] = len(traffic_lights)
            simulation_state.traffic_light_details = traffic_lights

            stage_start = time.perf_counter()
            if (
                self.construction_zone_shapes is None
                and simulator.env.static_adversity is not None
                and simulator.env.static_adversity.adversities is not None
            ):
                self.construction_zone_shapes = {}
                for adversity in simulator.env.static_adversity.adversities:
                    if isinstance(adversity, ConstructionAdversity):
                        lane_shape = traci.lane.getShape(adversity._lane_id)
                        if lane_shape:
                            lane_shape = interpolate_by_distance(lane_shape, 2.0)
                            lane_index = int(adversity._lane_id.split("_")[-1])
                            edge_id = traci.lane.getEdgeID(adversity._lane_id)
                            if lane_index == 0:
                                direction = 1
                            elif lane_index == traci.edge.getLaneNumber(edge_id) - 1:
                                direction = -1
                            else:
                                continue
                            construction_zone_shape = generate_construction_zone_shape(
                                lane_shape,
                                traci.lane.getWidth(adversity._lane_id),
                                direction,
                            )
                            self.construction_zone_shapes[adversity._lane_id] = (
                                construction_zone_shape
                            )
            profile["construction_zone"] = time.perf_counter() - stage_start
            simulation_state.construction_zone_details = self.construction_zone_shapes

            stage_start = time.perf_counter()
            state_json = simulation_state.model_dump_json()
            profile["json_dump"] = time.perf_counter() - stage_start
            profile["json_bytes"] = len(state_json)

            stage_start = time.perf_counter()
            self.redis_client.set(f"simulation:{self.simulation_uuid}:state", state_json)
            profile["redis_set"] = time.perf_counter() - stage_start

            stage_start = time.perf_counter()
            self.redis_client.expire(
                f"simulation:{self.simulation_uuid}:state", self.key_expiry
            )
            profile["redis_expire"] = time.perf_counter() - stage_start

            self.error_count = 0
            self.last_successful_operation = time.time()
            profile["total"] = time.perf_counter() - total_start
            return True

        except Exception as e:
            profile["result"] = "exception"
            profile["total"] = time.perf_counter() - total_start
            self.error_count += 1
            error_msg = str(e).lower()

            critical_errors = [
                "no network loaded",
                "connection lost",
                "traci",
                "sumo",
                "simulation crashed",
            ]

            is_critical = any(err in error_msg for err in critical_errors)

            self.logger.error(f"TeraSim error #{self.error_count}: {e}")

            if is_critical or self.error_count >= 3:
                self.logger.critical("TeraSim appears broken, stopping simulation")
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop",
                    f"terasim_error_{self.error_count}",
                    ex=300,
                )
                return False

            if time.time() - self.last_successful_operation > 300:
                self.logger.critical("TeraSim not responding for 5 minutes, stopping")
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop",
                    "terasim_timeout",
                    ex=300,
                )
                return False

            return True

    def _handle_agent_command(self, command_data):
        """Handle agent control commands.
        
        Args:
            command_data (str): The agent command data.
        """
        try:
            command = AgentCommand.model_validate_json(command_data.decode("utf-8"))
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
                        # Snap externally controlled vehicles to the closest lane. keepRoute=2 can
                        # leave the AV off-road, which makes SUMO report an empty lane id.
                        traci.vehicle.moveToXY(
                            command.agent_id, "", 0, x, y, command.data.get("sumo_angle", 0), 0
                        )

                        if "speed" in command.data:
                            traci.vehicle.setPreviousSpeed(command.agent_id, command.data["speed"])
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
            

                self.logger.info(f"Agent command executed: {command_data}")
                return True

        except Exception as e:
            self.logger.error(f"Error handling agent command: {e}")
            return False

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

    def _handle_pending_agent_commands(self):
        """Handle all pending agent commands in the queue."""
        if not self._check_simulation_status():
            return
        """Handle all pending agent commands in the queue"""
        try:
            # Process up to 100 commands per step to prevent infinite loops
            for _ in range(100):
                command_data = self.redis_client.lpop(
                    f"simulation:{self.simulation_uuid}:agent_commands"
                )
                if not command_data:
                    break

                self._handle_agent_command(command_data)
        except Exception as e:
            self.logger.error(f"Error handling pending agent commands: {e}")

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


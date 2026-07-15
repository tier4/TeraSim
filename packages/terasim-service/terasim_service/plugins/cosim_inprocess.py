"""TeraSimCoSimInProcessPlugin: the co-simulation plugin (single-process link).

The CARLA-facing co-sim loop (client thread) and the TeraSim simulation loop
(sim thread) live in ONE process and exchange commands/state as Python
objects. This replaced the transports of the earlier co-sim stages (Redis
lists polled by a FastAPI service, then gRPC RPCs between two processes),
both of which have been removed:

  Redis "control" key polling / Tick RPC   -> tick_async() + threading.Event
  Redis "agent_commands" list / RPC field  -> AgentCommand objects (no JSON)
  Redis "state" keys / RPC state_json      -> TickResult.state (dict, no JSON)

One tick_async() call = deliver this step's agent commands, run exactly one
SUMO step, publish the post-step state. The client thread and the sim loop
rendezvous through two events (_tick_requested / _step_done).

Threading contract: a SINGLE co-sim client thread, calling
tick_async() -> handle.result() strictly in that order (the next tick_async
only after the previous handle resolved). The published state dict is a fresh
snapshot each step and is never mutated by the plugin afterwards.

The simulation-state construction (_build_simulation_state) and agent-command
application (_apply_agent_command) used to live in the Redis-era
TeraSimCoSimPlugin base class shared by all transports; with the other
transports gone they are part of this class.
"""

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from logging.handlers import RotatingFileHandler
from typing import Optional

import numpy as np

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_nde_nade.adversity import ConstructionAdversity

from .base import BasePlugin
from ..utils import SimulationState, SUMOSignal, AgentCommand
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


@dataclass
class TickResult:
    """Snapshot of the co-sim state after a step (or at rest)."""

    status: str
    state: Optional[dict]  # SimulationState.model_dump(); None before the first build
    completed_sumo_time: float
    completed_tick_count: int


class TickHandle:
    """Future-like handle for one requested SUMO step.

    result() blocks until the sim thread finishes that step (or the
    simulation ends) and returns the post-step TickResult.
    """

    def __init__(self, plugin: "TeraSimCoSimInProcessPlugin", resolved: Optional[TickResult] = None):
        self._plugin = plugin
        self._resolved = resolved  # pre-resolved for pass-through (ended) calls

    def result(self, timeout: float = 300.0) -> TickResult:
        if self._resolved is not None:
            return self._resolved
        if not self._plugin._step_done.wait(timeout=timeout):
            raise TimeoutError(
                f"SUMO step did not complete within {timeout:.0f}s"
            )
        return self._plugin.get_result()


class TeraSimCoSimInProcessPlugin(BasePlugin):
    """Co-simulation plugin driven by a same-process co-sim client."""

    # Longest time function_before_env_step keeps waiting for a tick request
    # before auto-stopping.
    IDLE_TIMEOUT_S = 600.0

    # cadence (in steps) for pruning per-vehicle caches of departed ids
    CACHE_PRUNE_EVERY_STEPS = 1200

    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        base_dir: str = "output",
        auto_run: bool = False,
    ):
        """Initialize the Co-Simulation plugin.

        Args:
            simulation_uuid (str): Unique identifier for the simulation instance.
            plugin_config (dict, optional): Configuration for the plugin. Defaults to DEFAULT_COSIM_PLUGIN_CONFIG.
            base_dir (str, optional): Base directory for the log file. Defaults to "output".
            auto_run (bool, optional): Must stay False: this link is strictly
                lock-stepped (one tick_async = one SUMO step).
        """
        super().__init__(simulation_uuid, plugin_config)
        if auto_run:
            # auto_run would advance SUMO without tick requests; this link is
            # strictly lock-stepped, so reject it early.
            raise ValueError("TeraSimCoSimInProcessPlugin requires auto_run=False")
        self.base_dir = base_dir

        # Setup logging
        self.logger = self._setup_logger(base_dir)

        # This plugin logs on the per-step hot path while holding the GIL, so
        # DEBUG-level chatter (e.g. the per-command dump) stays off unless
        # explicitly requested; INFO keeps the step-finished measurement line.
        if os.getenv("TERASIM_COSIM_LOG_DEBUG", "") in ("", "0", "false", "no"):
            self.logger.setLevel(logging.INFO)

        # Maintain controlled agents in each step, assuming each agent can be controlled by only one command
        self.controlled_agents_each_step = set()

        # Cache construction zone shapes
        self.construction_zone_shapes = None

        # Initialize last orientations cache
        self.last_orientations = {}  # {vehicle_id: (last_orientation, last_time)}

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

        # lon/lat per agent costs one convertGeo (projection) per vehicle per
        # step, and the in-process consumer (CarlaCosim) converts coordinates
        # from x/y itself and never reads lon/lat, so skip it by default
        # (TERASIM_COSIM_STATE_LONLAT=1 re-enables it for external consumers
        # of the recorded state).
        self.state_lonlat_enabled = self._parse_bool_env(
            "TERASIM_COSIM_STATE_LONLAT", False
        )
        # Per-vehicle static attributes (length/width/height/type are constant
        # in SUMO) and the static half of the traffic-light details; both were
        # re-fetched/re-serialized every step.
        self._static_attr_cache = {}
        self._tls_static_cache = None
        self._cache_prune_countdown = self.CACHE_PRUNE_EVERY_STEPS

        # In-process rendezvous state (client thread <-> sim thread)
        self._lock = threading.Lock()
        self._status = "created"
        self._state = None  # dict (SimulationState.model_dump())
        self._completed_sumo_time = 0.0
        self._completed_tick_count = 0
        self._pending_commands = []  # list[AgentCommand]
        self._stop_requested = False
        self._ready = threading.Event()  # set once wait_for_tick is reached (or startup failed)
        self._tick_requested = threading.Event()
        self._step_done = threading.Event()
        self._client_serial = threading.Lock()  # serialize concurrent tick_async calls

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

    # ------------------------------------------------------------------
    # client-side API (called from the co-sim client thread)
    # ------------------------------------------------------------------
    def wait_until_ready(self, timeout: float) -> bool:
        """Block until the simulation reaches wait_for_tick (SUMO loaded).

        Returns True when the plugin is ready for tick_async; False on
        timeout or when the simulation already ended during startup.
        """
        if not self._ready.wait(timeout=timeout):
            return False
        with self._lock:
            return self._status == "wait_for_tick"

    def tick_async(self, commands) -> TickHandle:
        """Request one SUMO step (non-blocking).

        commands: list of dicts {agent_id, agent_type, command_type, data}
        (same shape the earlier transports carried as JSON). Returns a
        TickHandle whose .result(timeout) yields the post-step TickResult.
        """
        with self._client_serial:
            with self._lock:
                if self._status in ("finished", "error") or self._stop_requested:
                    return TickHandle(self, resolved=self._result_locked())
                self._pending_commands = [
                    AgentCommand.model_validate(c) for c in commands
                ]
            self._step_done.clear()
            self._tick_requested.set()
            return TickHandle(self)

    def get_result(self) -> TickResult:
        """Fetch the latest state without advancing the simulation."""
        with self._lock:
            return self._result_locked()

    def request_stop(self):
        """Ask the simulation loop to stop (idempotent, thread-safe)."""
        self.logger.info("Stop requested by the co-sim client")
        self._stop_requested = True

    def abort(self, status: str = "error"):
        """Mark the simulation as ended on behalf of a dead sim thread.

        Called by the runner when sim.run() raises: releases a client blocked
        in wait_until_ready()/result() so the process can shut down.
        """
        self._finish(status)
        self._ready.set()

    # ------------------------------------------------------------------
    # lifecycle hooks
    # ------------------------------------------------------------------
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

    def function_before_env_start(self, simulator: Simulator, ctx):
        self._set_status("initializing")
        self.logger.info(
            f"Simulation UUID: {self.simulation_uuid}, start initialization!"
        )
        return True

    def function_after_env_start(self, simulator: Simulator, ctx):
        try:
            # Build an initial state so the client can seed its render
            # pipeline (e.g. AV shape init) before the first tick.
            try:
                state = self._build_simulation_state(simulator)
                with self._lock:
                    self._state = state.model_dump()
            except Exception as e:
                self.logger.warning(f"Initial state build failed (non-fatal): {e}")
            self._set_status("wait_for_tick")
            self._ready.set()
            self.logger.info(
                f"Simulation UUID: {self.simulation_uuid}, finish initialization!"
            )
            return True
        except Exception as e:
            self.logger.exception(f"Unexpected error after start: {e}")
            self.abort("error")
            return False

    def function_before_env_step(self, simulator: Simulator, ctx):
        idle_start = time.time()
        while True:
            if self._stop_requested:
                self.logger.info("Stopping simulation")
                simulator.running = False  # stop the main loop
                return False
            if time.time() - idle_start > self.IDLE_TIMEOUT_S:
                self.logger.warning("No tick request for %.0fs, auto-stopping", self.IDLE_TIMEOUT_S)
                simulator.running = False
                return False
            if self._tick_requested.wait(timeout=0.1):
                self._tick_requested.clear()
                break

        # Apply the commands delivered with this tick request.
        with self._lock:
            commands = self._pending_commands
            self._pending_commands = []
        self.controlled_agents_each_step.clear()
        for command in commands:
            self._apply_agent_command(command)

        self._set_status("running")
        self.logger.debug("Simulation step started")
        return True

    def function_after_env_step(self, simulator: Simulator, ctx):
        try:
            state = self._build_simulation_state(simulator)
        except Exception as e:
            self.logger.exception(f"State build failed, stopping simulation: {e}")
            self._finish("error")
            return False
        completed_sumo_time = traci.simulation.getTime()
        with self._lock:
            self._state = state.model_dump()
            self._completed_sumo_time = completed_sumo_time
            self._completed_tick_count += 1
            self._status = "ticked"
            completed_tick_count = self._completed_tick_count
        self._step_done.set()
        # One line per step on purpose: with the RPC observation endpoint
        # gone, this log line (console handler prints asctime) is the external
        # interface for step-rate / clock-ratio / vehicle-count measurement.
        # vehicles= is the TOTAL SUMO vehicle count (the measurement x-axis; it
        # must not shrink when TERASIM_COSIM_STATE_FILTER trims the published
        # state); vehicles_state= is what actually went into the state.
        try:
            state_vehicle_count = state.agent_count.get("vehicle", -1)
        except Exception:
            state_vehicle_count = -1
        try:
            total_vehicle_count = traci.vehicle.getIDCount()
        except Exception:
            total_vehicle_count = state_vehicle_count
        self.logger.info(
            "Simulation step finished! completed_sumo_time=%s completed_tick_count=%s "
            "vehicles=%s vehicles_state=%s",
            completed_sumo_time,
            completed_tick_count,
            total_vehicle_count,
            state_vehicle_count,
        )
        return True

    def function_before_env_stop(self, simulator: Simulator, ctx):
        pass

    def function_after_env_stop(self, simulator: Simulator, ctx):
        self._finish("finished")
        self.logger.info(f"Simulation {self.simulation_uuid} finished!")

    # ------------------------------------------------------------------
    # simulation-state construction (shared with no other transport since
    # the Redis and gRPC paths were removed; kept factored for reuse)
    # ------------------------------------------------------------------
    def get_vehicle_vru_ids(self):
        """Get all vehicle and VRU IDs in the simulation."""
        all_ids = set(traci.vehicle.getIDList() + traci.person.getIDList())
        # Separate by type in one pass: construction objects, VRUs, and regular vehicles
        construction_ids, vru_ids, vehicle_ids = [], [], []
        for agent_id in all_ids:
            if agent_id.startswith("CONSTRUCTION_"):
                construction_ids.append(agent_id)
            elif "VRU" in agent_id:
                vru_ids.append(agent_id)
            else:
                vehicle_ids.append(agent_id)
        return vehicle_ids, vru_ids, construction_ids

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

    def _populate_lane_relative_position(self, vehicle_id, vehicle_state):
        """Fill the lane-relative fields of a vehicle-state dict (opt-in path)."""
        if not self.lane_relative_position_enabled:
            return

        lane_id = traci.vehicle.getLaneID(vehicle_id)
        if not lane_id:
            return
        lane_position = traci.vehicle.getLanePosition(vehicle_id)
        lateral_offset = traci.vehicle.getLateralLanePosition(vehicle_id)
        lane_shape = traci.lane.getShape(lane_id)
        reconstructed = reconstruct_position_from_lane_geometry(
            lane_shape,
            lane_position,
            lateral_offset,
            vehicle_state["z"],
        )

        vehicle_state["lane_id"] = lane_id
        vehicle_state["lane_position"] = lane_position
        vehicle_state["lateral_offset"] = lateral_offset
        if reconstructed is None:
            return
        (
            vehicle_state["reconstructed_x"],
            vehicle_state["reconstructed_y"],
            vehicle_state["reconstructed_z"],
        ) = reconstructed
        vehicle_state["reconstructed_position_valid"] = True

    def _build_simulation_state(self, simulator):
        """Collect the current simulation state from SUMO into a SimulationState.

        Pure state construction (no network I/O); raises on TraCI errors.
        """
        simulation_state = SimulationState()
        simulation_time = traci.simulation.getTime()
        simulation_state.simulation_time = simulation_time

        # Get all interested agent IDs
        all_vehicle_ids, vru_ids, construction_ids = self.get_vehicle_vru_ids()
        vehicle_ids, vehicle_position_cache = self._filter_vehicle_ids_for_state(
            all_vehicle_ids
        )
        simulation_state.agent_count = {
            "vehicle": len(vehicle_ids),
            "vru": len(vru_ids),
            "construction": len(construction_ids),
        }

        # Occasionally drop departed ids from the per-vehicle caches (they are
        # keyed by SUMO id and would otherwise grow for the whole run).
        self._cache_prune_countdown -= 1
        if self._cache_prune_countdown <= 0:
            self._cache_prune_countdown = self.CACHE_PRUNE_EVERY_STEPS
            alive = set(all_vehicle_ids)
            alive.update(vru_ids)
            for cache in (self.last_orientations, self._static_attr_cache):
                for stale_id in [key for key in cache if key not in alive]:
                    del cache[stale_id]

        # Add vehicle states (plain dicts in the AgentStateSimplified shape;
        # scalar math via the math module — numpy scalar ufuncs are several
        # times slower and this loop runs per vehicle per step).
        lonlat_enabled = self.state_lonlat_enabled
        static_attrs = self._static_attr_cache
        last_orientations = self.last_orientations
        vehicles = {}
        for vid in vehicle_ids:
            position = vehicle_position_cache.get(vid)
            if position is None:
                position = traci.vehicle.getPosition3D(vid)
            x, y, z = position
            if lonlat_enabled:
                lon, lat = traci.simulation.convertGeo(x, y)
            else:
                lon = lat = 0.0
            sumo_angle = traci.vehicle.getAngle(vid)
            orientation = math.radians((90.0 - sumo_angle) % 360.0)
            static = static_attrs.get(vid)
            if static is None:
                static = (
                    traci.vehicle.getLength(vid),
                    traci.vehicle.getWidth(vid),
                    traci.vehicle.getHeight(vid),
                    traci.vehicle.getTypeID(vid),
                )
                static_attrs[vid] = static
            last_orientation, last_time = last_orientations.get(vid, (orientation, simulation_time))
            dt = simulation_time - last_time
            if dt > 0:
                dtheta = orientation - last_orientation
                angular_velocity = math.atan2(math.sin(dtheta), math.cos(dtheta)) / dt
            else:
                angular_velocity = 0.0
            last_orientations[vid] = (orientation, simulation_time)
            vehicle_state = {
                "x": x,
                "y": y,
                "z": z,
                "lane_id": "",
                "lane_position": 0.0,
                "lateral_offset": 0.0,
                "reconstructed_x": 0.0,
                "reconstructed_y": 0.0,
                "reconstructed_z": 0.0,
                "reconstructed_position_valid": False,
                "lon": lon,
                "lat": lat,
                "sumo_angle": sumo_angle,
                "length": static[0],
                "width": static[1],
                "height": static[2],
                "speed": traci.vehicle.getSpeed(vid),
                "orientation": orientation,
                "acceleration": traci.vehicle.getAcceleration(vid),
                "angular_velocity": angular_velocity,
                "type": static[3],
            }
            if self.lane_relative_position_enabled:
                self._populate_lane_relative_position(vid, vehicle_state)
            vehicles[vid] = vehicle_state

        simulation_state.agent_details["vehicle"] = vehicles

        # Add VRU states
        # Get current vehicle and person lists to determine actual object type
        current_vehicle_list = traci.vehicle.getIDList()
        current_person_list = traci.person.getIDList()

        vrus = {}
        for vru_id in vru_ids:
            # Determine if this VRU is actually a vehicle or person
            if vru_id in current_vehicle_list:
                # VRU is actually a vehicle (disguised as pedestrian)
                domain = traci.vehicle
            elif vru_id in current_person_list:
                # VRU is actually a person
                domain = traci.person
            else:
                # VRU ID not found in either list, log warning and skip
                self.logger.warning(f"VRU ID {vru_id} not found in vehicle or person lists, skipping")
                continue

            x, y, z = domain.getPosition3D(vru_id)
            if lonlat_enabled:
                lon, lat = traci.simulation.convertGeo(x, y)
            else:
                lon = lat = 0.0
            sumo_angle = domain.getAngle(vru_id)
            orientation = math.radians((90.0 - sumo_angle) % 360.0)
            angular_velocity = 0.0
            if domain is traci.vehicle:
                last_orientation, last_time = last_orientations.get(vru_id, (orientation, simulation_time))
                dt = simulation_time - last_time
                if dt > 0:
                    dtheta = orientation - last_orientation
                    angular_velocity = math.atan2(math.sin(dtheta), math.cos(dtheta)) / dt
                last_orientations[vru_id] = (orientation, simulation_time)
                acceleration = domain.getAcceleration(vru_id)
            else:
                acceleration = (
                    domain.getAcceleration(vru_id)
                    if hasattr(domain, "getAcceleration")
                    else 0.0
                )

            vrus[vru_id] = {
                "x": x,
                "y": y,
                "z": z,
                "lane_id": "",
                "lane_position": 0.0,
                "lateral_offset": 0.0,
                "reconstructed_x": 0.0,
                "reconstructed_y": 0.0,
                "reconstructed_z": 0.0,
                "reconstructed_position_valid": False,
                "lon": lon,
                "lat": lat,
                "sumo_angle": sumo_angle,
                "length": domain.getLength(vru_id),
                "width": domain.getWidth(vru_id),
                "height": domain.getHeight(vru_id),
                "speed": domain.getSpeed(vru_id),
                "orientation": orientation,
                "acceleration": acceleration,
                "angular_velocity": angular_velocity,
                "type": domain.getTypeID(vru_id),
            }

        simulation_state.agent_details["vru"] = vrus

        # Add construction objects
        construction_objects = {}
        for cid in construction_ids:
            x, y, z = traci.vehicle.getPosition3D(cid)
            if lonlat_enabled:
                lon, lat = traci.simulation.convertGeo(x, y)
            else:
                lon = lat = 0.0
            sumo_angle = traci.vehicle.getAngle(cid)
            construction_objects[cid] = {
                "x": x,
                "y": y,
                "z": z,
                "lane_id": "",
                "lane_position": 0.0,
                "lateral_offset": 0.0,
                "reconstructed_x": 0.0,
                "reconstructed_y": 0.0,
                "reconstructed_z": 0.0,
                "reconstructed_position_valid": False,
                "lon": lon,
                "lat": lat,
                "sumo_angle": sumo_angle,
                "length": traci.vehicle.getLength(cid),
                "width": traci.vehicle.getWidth(cid),
                "height": traci.vehicle.getHeight(cid),
                "speed": traci.vehicle.getSpeed(cid),
                "orientation": math.radians((90.0 - sumo_angle) % 360.0),
                "acceleration": traci.vehicle.getAcceleration(cid),
                "angular_velocity": 0.0,
                "type": traci.vehicle.getTypeID(cid),
            }

        simulation_state.construction_objects = construction_objects

        # Add traffic light states. The program/parameter block is static per
        # TLS, so it is resolved from the SUMO net and JSON-encoded exactly
        # once; only the current signal string changes per step.
        if self._tls_static_cache is None:
            self._tls_static_cache = {}
            for tl_id in traci.trafficlight.getIDList():
                tls_information = {
                    "programs": {}
                }
                tls = self.simulator.sumo_net.getTLS(tl_id)
                programs = tls.getPrograms()
                for program_id, program in programs.items():
                    # Get the program parameters
                    program_parameters = program.getParams()
                    tls_information["programs"][program_id] = {
                        "parameters": program_parameters
                    }
                self._tls_static_cache[tl_id] = json.dumps(tls_information)

        traffic_lights = {}
        for tl_id, information in self._tls_static_cache.items():
            sumo_signal = SUMOSignal()
            sumo_signal.x, sumo_signal.y = 0, 0
            sumo_signal.tls = traci.trafficlight.getRedYellowGreenState(tl_id)
            sumo_signal.information = information
            traffic_lights[tl_id] = sumo_signal

        simulation_state.traffic_light_details = traffic_lights

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

    # ------------------------------------------------------------------
    # agent command application
    # ------------------------------------------------------------------
    def _apply_agent_command(self, command):
        """Apply a parsed AgentCommand to the running simulation."""
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
                        # 3-cosim fix: keepRoute=0 (snap to closest lane in the network),
                        # not 2 (free / off-road). With keepRoute=2 an externally-driven
                        # vehicle (e.g. the Autoware ego mirrored as SUMO "AV" via control_av)
                        # lands slightly off the lane centerline -> getLaneID()=="" -> it drops
                        # out of the AV context subscription -> NADE stops controlling traffic
                        # around it -> background vehicles no longer yield and rear-end the ego.
                        # keepRoute=0 keeps the AV on a lane so SUMO traffic avoids it.
                        traci.vehicle.moveToXY(
                            command.agent_id, "", 0, x, y, command.data.get("sumo_angle", 0), 0
                        )

                        # 3-cosim fix (dense maps, e.g. Odaiba): right after moveToXY, append one
                        # successor edge so the externally-driven AV's route is never a single
                        # terminal edge. With keepRoute=0 a dense network can map the AV onto an
                        # off-route edge, collapsing its route to that one edge; the AV then reaches
                        # that edge's end, SUMO retires it as "arrived", NADE stops with
                        # finish_reason "AV_left", and the cosim crashes. The AV's pose is driven
                        # entirely by moveToXY (it mirrors the Autoware ego), so this 2-edge route is
                        # only a decoy to keep it alive -- NOT a fixed plan, which is correct because
                        # the Autoware ego chooses its path dynamically.
                        # (No getIDList membership pre-check: materializing the
                        # full id tuple every step is O(total vehicles), and the
                        # try/except below already tolerates a missing AV.)
                        if command.agent_id == "AV":
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


                if self.logger.isEnabledFor(logging.DEBUG):
                    # Guarded: the model_dump_json() alone is measurable at one
                    # command per step, and this fires on the hot path.
                    self.logger.debug(f"Agent command executed: {command.model_dump_json()}")
                return True

        except Exception as e:
            self.logger.error(f"Error handling agent command: {e}")
            return False

    # ------------------------------------------------------------------
    # internal state helpers
    # ------------------------------------------------------------------
    def _set_status(self, status: str):
        with self._lock:
            self._status = status

    def _finish(self, status: str):
        with self._lock:
            self._status = status
        # Release a client waiting on a step that will never come.
        self._step_done.set()

    def _result_locked(self) -> TickResult:
        return TickResult(
            status=self._status,
            state=self._state,
            completed_sumo_time=self._completed_sumo_time,
            completed_tick_count=self._completed_tick_count,
        )

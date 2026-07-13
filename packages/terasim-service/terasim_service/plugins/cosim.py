import json
import logging
import math
import os
from logging.handlers import RotatingFileHandler
import numpy as np
import redis
from redis.exceptions import RedisError
import time
import subprocess
from pathlib import Path

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_nde_nade.adversity import ConstructionAdversity

from .base import BasePlugin, DEFAULT_REDIS_CONFIG

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


class TeraSimCoSimPlugin(BasePlugin):
    # lon/lat in the published state (see state_lonlat_enabled in __init__)
    STATE_LONLAT_DEFAULT = True
    # cadence (in steps) for pruning per-vehicle caches of departed ids
    CACHE_PRUNE_EVERY_STEPS = 1200

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
        # step. The CARLA co-sim client converts from x/y itself and never
        # reads them, so the in-process plugin turns them off by default
        # (STATE_LONLAT_DEFAULT); the Redis path keeps them for external
        # consumers. Env var overrides either way.
        self.state_lonlat_enabled = self._parse_bool_env(
            "TERASIM_COSIM_STATE_LONLAT", self.STATE_LONLAT_DEFAULT
        )
        # Per-vehicle static attributes (length/width/height/type are constant
        # in SUMO) and the static half of the traffic-light details; both were
        # re-fetched/re-serialized every step.
        self._static_attr_cache = {}
        self._tls_static_cache = None
        self._cache_prune_countdown = self.CACHE_PRUNE_EVERY_STEPS

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

        Pure state construction (no Redis/network I/O); raises on TraCI errors.
        Shared by the Redis-backed `_write_simulation_state` and the direct
        (gRPC) plugin, which returns the result over its Tick/GetState RPC
        instead of writing to Redis.
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
        """Handle agent control commands delivered as JSON bytes.

        Args:
            command_data (bytes): The agent command data (JSON wire format,
                as stored in the Redis agent_commands list).
        """
        try:
            command = AgentCommand.model_validate_json(command_data.decode("utf-8"))
        except Exception as e:
            self.logger.error(f"Error handling agent command: {e}")
            return False
        return self._apply_agent_command(command)

    def _apply_agent_command(self, command):
        """Apply a parsed AgentCommand to the running simulation.

        Extracted from _handle_agent_command so callers that already hold an
        AgentCommand object (e.g. an in-process co-sim client) can skip the
        JSON round-trip; every transport shares this implementation.
        """
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



import time
import json
import math
import os
import re
import carla
import random
import xml.etree.ElementTree as ET
import yaml
import statistics

from .ackermann_control import (
    AckermannTuning,
    compute_ackermann_control_values,
    horizontal_speed,
)
from .tools import (
    carla_to_sumo,
    create_bike_blueprint,
    create_bikeandmotor_blueprint,
    create_motor_blueprint,
    create_pedestrian_blueprint,
    create_police_car_blueprint,
    create_vehicle_blueprint,
    destroy_all_actors,
    draw_text,
    get_actor_id_from_attribute,
    log_spawn_actor_failure,
    sumo_point_to_carla,
    sumo_to_carla,
    spawn_actor,
)
from ..service import (
    control_agent,
    control_agents_batch,
    start_terasim,
    stop_terasim,
    tick_terasim,
    get_terasim_status,
    get_terasim_states,
)

AV_SUMO_ID = "AV"
SUMO_CARLA_TLS_LINK_PREFIX = "linkSignalID:"
VEHICLE_CONTROL_MODE_TELEPORT = "teleport"
VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS = "ackermann_physics"
VEHICLE_CONTROL_MODES = {
    VEHICLE_CONTROL_MODE_TELEPORT,
    VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS,
}
ACKERMANN_FEEDBACK_MODE_OFF = "off"
ACKERMANN_FEEDBACK_MODE_SHADOW = "shadow"
ACKERMANN_FEEDBACK_MODE_APPLY = "apply"
ACKERMANN_FEEDBACK_MODES = {
    ACKERMANN_FEEDBACK_MODE_OFF,
    ACKERMANN_FEEDBACK_MODE_SHADOW,
    ACKERMANN_FEEDBACK_MODE_APPLY,
}


def _env_float(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.", flush=True)
        return default


def _env_int(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return int(value)
    except ValueError:
        print(f"Warning: invalid {name}={value!r}; using {default}.", flush=True)
        return default


def _env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


class CarlaCosim(object):
    def __init__(self, args):
        self.args = args

        self.client = carla.Client(args.carla_host, args.carla_port)
        self.client.set_timeout(getattr(args, 'carla_timeout', 10.0))

        self.world = self.client.get_world()
        if args.map_name:
            print(f"Loading map {args.map_name}")
            try:
                self.world = self.client.load_world(args.map_name)
            except:
                print(f"Map {args.map_name} not found. Loading default map.")
        else:
            print("No map name provided. Loading default map.")

        self.traffic_lights = self.world.get_actors().filter("traffic.traffic_light")
        for traffic_light in self.traffic_lights:
            traffic_light.set_state(carla.TrafficLightState.Off)
            traffic_light.freeze(True)

        self.control_av = args.control_av
        self.initialize_av = False
        self.av_shape = []
        self.async_mode = args.async_mode
        self.step_length = args.step_length
        self._spawn_failures = {}
        self._missing_angle_warnings = set()
        self._invalid_location_warnings = set()
        requested_vehicle_control_mode = (
            getattr(args, "vehicle_control_mode", None)
            or os.environ.get("CARLA_COSIM_VEHICLE_CONTROL_MODE")
            or VEHICLE_CONTROL_MODE_TELEPORT
        )
        self.vehicle_control_mode = str(requested_vehicle_control_mode).strip().lower()
        if self.vehicle_control_mode not in VEHICLE_CONTROL_MODES:
            print(
                f"Warning: invalid vehicle control mode {requested_vehicle_control_mode!r}; "
                f"using {VEHICLE_CONTROL_MODE_TELEPORT}.",
                flush=True,
            )
            self.vehicle_control_mode = VEHICLE_CONTROL_MODE_TELEPORT
        self.ackermann_physics_enabled = (
            self.vehicle_control_mode == VEHICLE_CONTROL_MODE_ACKERMANN_PHYSICS
        )
        requested_feedback_mode = os.environ.get(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE", ACKERMANN_FEEDBACK_MODE_OFF
        )
        self.ackermann_feedback_mode = str(requested_feedback_mode).strip().lower()
        if self.ackermann_feedback_mode not in ACKERMANN_FEEDBACK_MODES:
            raise ValueError(
                "CARLA_COSIM_ACKERMANN_FEEDBACK_MODE must be one of "
                f"{sorted(ACKERMANN_FEEDBACK_MODES)}, got {requested_feedback_mode!r}"
            )
        feedback_actor_value = os.environ.get("CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS", "")
        self.ackermann_feedback_actor_ids = {
            actor_id.strip() for actor_id in feedback_actor_value.split(",") if actor_id.strip()
        }
        self.ackermann_feedback_all_background_actors = "*" in self.ackermann_feedback_actor_ids
        if self.ackermann_feedback_mode != ACKERMANN_FEEDBACK_MODE_OFF:
            if not self.ackermann_physics_enabled:
                raise ValueError(
                    "Ackermann feedback requires vehicle_control_mode=ackermann_physics"
                )
            if self.async_mode:
                raise ValueError("Ackermann feedback requires synchronous CARLA mode")
            if getattr(args, "passive_tick", False):
                raise ValueError("Ackermann feedback is incompatible with passive_tick")
            if self.control_av:
                raise ValueError(
                    "Ackermann feedback and control_av cannot own the AV at the same time"
                )
        self.ackermann_feedback_apply_enabled = (
            self.ackermann_feedback_mode == ACKERMANN_FEEDBACK_MODE_APPLY
            and bool(self.ackermann_feedback_actor_ids)
        )
        self.ackermann_feedback_shadow_enabled = (
            self.ackermann_feedback_mode == ACKERMANN_FEEDBACK_MODE_SHADOW
            and bool(self.ackermann_feedback_actor_ids)
        )
        self._ackermann_feedback_state = {}
        self.ackermann_feedback_log_records = _env_bool(
            "CARLA_COSIM_ACKERMANN_FEEDBACK_LOG_RECORDS", True
        )
        self._ackermann_feedback_actor_index = {}
        self._ackermann_feedback_candidate_actor_ids = set()
        self._ackermann_actor_state = {}
        self._initial_terasim_state_pending = True
        self._last_completed_terasim_tick_count = None
        self.terasim_states = {}
        self.ackermann_feedback_speed_horizon = max(
            self.step_length,
            _env_float("CARLA_COSIM_ACKERMANN_FEEDBACK_SPEED_HORIZON", 1.0),
        )
        self.ackermann_feedback_ack_max_frame_lag = max(
            0, _env_int("CARLA_COSIM_ACKERMANN_FEEDBACK_ACK_MAX_FRAME_LAG", 2)
        )
        self.ackermann_feedback_ack_failure_limit = max(
            1, _env_int("CARLA_COSIM_ACKERMANN_FEEDBACK_ACK_FAILURE_LIMIT", 3)
        )
        self.ackermann_tuning = AckermannTuning(
            wheel_base=max(0.1, _env_float("CARLA_COSIM_ACKERMANN_WHEEL_BASE", 2.8)),
            max_steer_rad=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_STEER_RAD", 0.6)),
            max_steer_rate_rad_s=max(
                0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_STEER_RATE_RAD_S", 0.6)
            ),
            kp_speed=_env_float("CARLA_COSIM_ACKERMANN_KP_SPEED", 0.8),
            kp_position=_env_float("CARLA_COSIM_ACKERMANN_KP_POSITION", 0.15),
            max_accel=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_ACCEL", 3.0)),
            max_decel=max(0.0, _env_float("CARLA_COSIM_ACKERMANN_MAX_DECEL", 6.0)),
        )
        self.ackermann_warn_error_m = max(
            0.0, _env_float("CARLA_COSIM_ACKERMANN_WARN_ERROR_M", 3.0)
        )
        self.ackermann_snap_error_m = max(
            self.ackermann_warn_error_m,
            _env_float("CARLA_COSIM_ACKERMANN_SNAP_ERROR_M", 8.0),
        )
        self.ackermann_warning_interval = max(
            0.0, _env_float("CARLA_COSIM_ACKERMANN_WARNING_INTERVAL", 2.0)
        )
        if self.ackermann_feedback_mode != ACKERMANN_FEEDBACK_MODE_OFF:
            print(
                "CARLA-to-TeraSim Ackermann feedback configured: "
                f"mode={self.ackermann_feedback_mode} "
                f"actors={sorted(self.ackermann_feedback_actor_ids)!r}.",
                flush=True,
            )
        if self.ackermann_physics_enabled:
            print("CARLA co-sim Ackermann physics vehicle control enabled.", flush=True)
        self.use_lane_relative_position = _env_bool(
            "CARLA_COSIM_USE_LANE_RELATIVE_POSITION",
            bool(getattr(args, "use_lane_relative_position", False)),
        )
        if self.use_lane_relative_position:
            print("CARLA co-sim lane-relative reconstructed positions enabled.", flush=True)
        self.spawn_failure_backoff_seconds = max(
            0.0,
            _env_float("CARLA_COSIM_SPAWN_FAILURE_BACKOFF_SECONDS", 5.0),
        )
        self.spawn_failure_backoff_max_seconds = max(
            self.spawn_failure_backoff_seconds,
            _env_float("CARLA_COSIM_SPAWN_FAILURE_BACKOFF_MAX_SECONDS", 30.0),
        )
        self.batch_transform_enabled = _env_bool("CARLA_COSIM_BATCH_TRANSFORM", False)
        if self.batch_transform_enabled:
            print("CARLA co-sim ApplyTransform batching enabled.", flush=True)
        self.batch_spawn_enabled = _env_bool("CARLA_COSIM_BATCH_SPAWN", False)
        if self.batch_spawn_enabled:
            print("CARLA co-sim SpawnActor batching enabled.", flush=True)
        self.spawn_z_clearance = max(
            0.0,
            _env_float("CARLA_COSIM_SPAWN_Z_CLEARANCE", self.SPAWN_Z_CLEARANCE),
        )
        print(
            f"CARLA co-sim spawn Z clearance: {self.spawn_z_clearance:.1f}m.",
            flush=True,
        )
        self.actor_filter_enabled = _env_bool("CARLA_COSIM_ACTOR_FILTER", False)
        self.actor_filter_center_id = (
            os.environ.get("CARLA_COSIM_ACTOR_FILTER_CENTER_ID") or AV_SUMO_ID
        )
        self.actor_filter_radius = max(
            0.0,
            _env_float("CARLA_COSIM_ACTOR_FILTER_RADIUS", 300.0),
        )
        self._actor_filter_missing_center_warned = False
        if self.actor_filter_enabled:
            print(
                "CARLA co-sim actor radius filter enabled: "
                f"center={self.actor_filter_center_id} "
                f"radius={self.actor_filter_radius:.1f}m.",
                flush=True,
            )

        self.vehicle_blueprints = create_vehicle_blueprint(self.world)
        self.motor_blueprints = create_motor_blueprint(self.world)
        self.pedestrian_blueprints = create_pedestrian_blueprint(self.world)
        self.police_car_blueprints = create_police_car_blueprint(self.world)
        self.bike_blueprints = create_bike_blueprint(self.world)
        self.bikeandmotor_blueprints = create_bikeandmotor_blueprint(self.world)

        # self.sync_cosim_construction_zone_to_carla()

        # start / connect to TeraSim
        self.direct_link = None
        self._direct_tick_future = None
        self._direct_prev_state = None
        direct_addr = getattr(args, "direct_addr", None)
        if direct_addr:
            # Direct (gRPC) mode: no Redis/FastAPI. The TeraSim runner
            # (terasim_service.run_direct) already owns the simulation; connect
            # and wait until it reaches wait_for_tick.
            from .direct_link import DirectLink, parse_state_json

            self.direct_link = DirectLink(direct_addr)
            # Seed the render pipeline with the initial (post-warmup) state so
            # the first tick behaves like the polling path (AV shape init etc.).
            self._direct_prev_state = parse_state_json(
                self.direct_link.get_state().state_json
            )
            self.terasim = {"simulation_id": "direct"}
        else:
            terasim_init_command = {
                "config_file": args.terasim_config,
                "auto_run": False,
            }
            self.terasim = start_terasim(args.terasim_host, args.terasim_port, terasim_init_command)
            while True:
                terasim_status = get_terasim_status(args.terasim_host, args.terasim_port, self.terasim["simulation_id"])
                if terasim_status.get("status", None) == "wait_for_tick":
                    break
                time.sleep(0.1)

        # Auto-calibrate SUMO-CARLA coordinate transformation
        self.sumo_carla_offset = [0.0, 0.0]
        self._coord_transformer = None
        self._sumo_net_offset = [0.0, 0.0]
        self._xodr_origin_utm = [0.0, 0.0]
        net_file = self._get_net_file_from_config(args.terasim_config)
        if net_file:
            result = self._calibrate_sumo_carla_offset(net_file)
            if result is not None:
                # Offset-based mode
                self.sumo_carla_offset = result
                print(f"SUMO-CARLA coordinate offset: dx={self.sumo_carla_offset[0]:.2f}, dy={self.sumo_carla_offset[1]:.2f}")
            else:
                print("Using projection-based coordinate transformation")

    @staticmethod
    def _get_net_file_from_config(config_path):
        """Extract SUMO net file path from the TeraSim scenario YAML config."""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            # Try input.sumo_net_file first, then environment.parameters.sumo_net_file_path
            net_file = config.get('input', {}).get('sumo_net_file')
            if not net_file:
                net_file = config.get('environment', {}).get('parameters', {}).get('sumo_net_file_path')
            return net_file
        except Exception as e:
            print(f"Warning: Could not read config for net file path: {e}")
            return None

    @staticmethod
    def _parse_xodr_origin(xodr_proj):
        """Extract lat_0/lon_0 and UTM zone from an xodr geoReference proj string.
        Returns (origin_lat, origin_lon, utm_zone) or (None, None, None) if not parseable.
        """
        import re
        lat_0 = lon_0 = utm_zone = None
        m = re.search(r'\+lat_0=([0-9.eE+-]+)', xodr_proj)
        if m:
            lat_0 = float(m.group(1))
        m = re.search(r'\+lon_0=([0-9.eE+-]+)', xodr_proj)
        if m:
            lon_0 = float(m.group(1))
        m = re.search(r'\+zone=(\d+)', xodr_proj)
        if m:
            utm_zone = int(m.group(1))
        return lat_0, lon_0, utm_zone

    def _calibrate_sumo_carla_offset(self, net_file):
        """Build a coordinate transformer between SUMO net.xml and CARLA (xodr) coordinate systems.

        OpenDRIVE files from Lanelet2 pipelines use coordinates that are local offsets from the
        geoReference origin (lat_0, lon_0) projected into standard UTM. pyproj ignores lat_0/lon_0
        for +proj=utm, so we handle this by:
        1. Detecting the SUMO coordinate system (EPSG:3857 for Lanelet2 conversions)
        2. Converting SUMO CRS -> standard UTM (matching xodr zone)
        3. Subtracting the geoReference origin (projected to the same UTM) to get xodr-local coords

        Returns [offset_x, offset_y] for simple offset mode, or sets self._coord_transformer
        for full projection-based conversion (returns None).
        """
        # Parse SUMO net.xml <location> for projection info
        sumo_proj = None
        sumo_net_offset = [0.0, 0.0]
        try:
            tree = ET.parse(net_file)
            root = tree.getroot()
            loc_elem = root.find('.//location')
            if loc_elem is not None:
                sumo_proj = loc_elem.get('projParameter', '!')
                offset_str = loc_elem.get('netOffset', '0.00,0.00')
                parts = offset_str.split(',')
                sumo_net_offset = [float(parts[0]), float(parts[1])]
                print(f"SUMO net.xml: projParameter='{sumo_proj}', netOffset={sumo_net_offset}")
        except Exception as e:
            print(f"Warning: Could not parse SUMO net file {net_file}: {e}")

        # Get xodr geoReference from CARLA map
        xodr_proj = None
        try:
            opendrive_str = self.world.get_map().to_opendrive()
            xodr_tree = ET.fromstring(opendrive_str)
            geo_elem = xodr_tree.find('.//geoReference')
            if geo_elem is not None and geo_elem.text:
                xodr_proj = geo_elem.text.strip()
                print(f"CARLA xodr geoReference: '{xodr_proj}'")
        except Exception as e:
            print(f"Warning: Could not get xodr geoReference from CARLA: {e}")

        # Attempt projection-based transformation with origin offset handling
        if xodr_proj:
            try:
                import pyproj

                # Parse xodr origin (lat_0, lon_0) and UTM zone
                origin_lat, origin_lon, utm_zone = self._parse_xodr_origin(xodr_proj)

                # Determine SUMO CRS
                sumo_crs = None
                if sumo_proj and sumo_proj != '!':
                    sumo_crs = pyproj.CRS(sumo_proj)
                elif sumo_proj == '!':
                    # Detect CRS empirically from coordinate ranges
                    tree = ET.parse(net_file)
                    root = tree.getroot()
                    conv_boundary = root.find('.//location').get('convBoundary', '')
                    cb_parts = conv_boundary.split(',')
                    sample_x = (float(cb_parts[0]) + float(cb_parts[2])) / 2
                    sample_y = (float(cb_parts[1]) + float(cb_parts[3])) / 2

                    wgs84 = pyproj.CRS('EPSG:4326')
                    for crs_code in ['EPSG:3857', 'EPSG:32654', 'EPSG:6677']:
                        try:
                            candidate = pyproj.CRS(crs_code)
                            to_wgs84 = pyproj.Transformer.from_crs(candidate, wgs84, always_xy=True)
                            lon, lat = to_wgs84.transform(sample_x, sample_y)
                            if 100.0 < lon < 180.0 and -60.0 < lat < 85.0:
                                sumo_crs = candidate
                                print(f"Detected SUMO CRS as {crs_code} (sample -> lon={lon:.4f}, lat={lat:.4f})")
                                break
                        except Exception:
                            continue

                if sumo_crs is None:
                    print("Warning: Could not determine SUMO CRS. Falling back to empirical calibration.")
                    return self._empirical_calibration(net_file)

                # Build transformer: SUMO CRS -> standard UTM (same zone as xodr)
                if utm_zone:
                    utm_crs = pyproj.CRS(f'EPSG:326{utm_zone:02d}')
                else:
                    # Default to UTM zone from xodr proj string
                    utm_crs = pyproj.CRS(xodr_proj)

                self._coord_transformer = pyproj.Transformer.from_crs(sumo_crs, utm_crs, always_xy=True)
                self._sumo_net_offset = sumo_net_offset

                # Compute xodr origin in standard UTM
                self._xodr_origin_utm = [0.0, 0.0]
                if origin_lat is not None and origin_lon is not None:
                    wgs84 = pyproj.CRS('EPSG:4326')
                    to_utm = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)
                    ox, oy = to_utm.transform(origin_lon, origin_lat)
                    self._xodr_origin_utm = [ox, oy]
                    print(f"xodr origin ({origin_lat:.6f}, {origin_lon:.6f}) in UTM: ({ox:.2f}, {oy:.2f})")

                print(f"Using projection-based transform: SUMO -> UTM{utm_zone} - origin")
                return None  # Signal to use transformer instead of offset

            except Exception as e:
                print(f"Warning: Projection-based calibration failed: {e}")
                import traceback
                traceback.print_exc()

        # Fallback: empirical median-based calibration
        return self._empirical_calibration(net_file)

    def _empirical_calibration(self, net_file):
        """Fallback: compute offset by comparing matching road coordinates."""
        sumo_edges = {}
        try:
            tree = ET.parse(net_file)
            root = tree.getroot()
            for edge_elem in root.iter('edge'):
                edge_id = edge_elem.get('id', '')
                if edge_id.startswith(':'):
                    continue
                for lane_elem in edge_elem.iter('lane'):
                    shape_str = lane_elem.get('shape', '')
                    if shape_str:
                        points = [tuple(map(float, p.split(','))) for p in shape_str.split()]
                        mid = points[len(points) // 2]
                        sumo_edges[edge_id] = (mid[0], mid[1])
                        break
        except Exception as e:
            print(f"Warning: Could not parse net file: {e}")
            return [0.0, 0.0]

        carla_roads = {}
        try:
            for w in self.world.get_map().generate_waypoints(200.0):
                rid = str(w.road_id)
                if rid not in carla_roads:
                    carla_roads[rid] = (w.transform.location.x, w.transform.location.y)
        except Exception as e:
            print(f"Warning: Could not get CARLA waypoints: {e}")
            return [0.0, 0.0]

        dxs, dys = [], []
        for edge_id, (sx, sy) in sumo_edges.items():
            if edge_id in carla_roads:
                cx, cy = carla_roads[edge_id]
                dxs.append(cx - sx)
                dys.append(cy + sy)

        if len(dxs) < 10:
            print(f"Warning: Only {len(dxs)} matching roads. Offset may be inaccurate.")
            if not dxs:
                return [0.0, 0.0]

        offset_x = statistics.median(dxs)
        offset_y = statistics.median(dys)
        print(f"Empirical calibration from {len(dxs)} matching roads")
        return [offset_x, offset_y]

    def _wait_for_terasim_step(self):
        while True:
            response = get_terasim_status(
                self.args.terasim_host,
                self.args.terasim_port,
                self.terasim["simulation_id"],
            )
            status = response.get("status")
            if status is None:
                print("TeraSim status is None. Exiting...")
                return None

            completed_tick_count = response.get("completed_tick_count")
            if completed_tick_count is not None:
                try:
                    completed_tick_count = int(completed_tick_count)
                except (TypeError, ValueError):
                    completed_tick_count = None

            if status == "wait_for_tick" and self._initial_terasim_state_pending:
                self._initial_terasim_state_pending = False
                if completed_tick_count is not None:
                    self._last_completed_terasim_tick_count = completed_tick_count
                return status

            if (
                status == "ticked"
                and completed_tick_count is not None
                and (
                    self._last_completed_terasim_tick_count is None
                    or completed_tick_count > self._last_completed_terasim_tick_count
                )
            ):
                self._initial_terasim_state_pending = False
                self._last_completed_terasim_tick_count = completed_tick_count
                return status
            time.sleep(0.05)

    def _tick_ackermann_feedback_apply_http(self):
        if self._wait_for_terasim_step() is None:
            return False

        self.sync_cosim_actor_to_carla()
        if not getattr(self.args, "skip_tls", False):
            self.sync_cosim_tls_to_carla()
        self.world.tick()
        self.sync_carla_ackermann_feedback_to_cosim()
        tick_terasim(
            self.args.terasim_host,
            self.args.terasim_port,
            self.terasim["simulation_id"],
        )
        return True

    def _tick_ackermann_feedback_apply_direct(self):
        import grpc as _grpc

        from .direct_link import parse_state_json

        if self._direct_tick_future is not None:
            try:
                response = self._direct_tick_future.result(timeout=300.0)
            except _grpc.RpcError as exc:
                print(f"TeraSim direct tick failed: {exc}. Exiting...")
                return False
            if response.status in ("finished", "error"):
                print(f"TeraSim ended (status={response.status}). Exiting...")
                return False
            state = parse_state_json(response.state_json)
            if state is not None:
                self._direct_prev_state = state

        if self._direct_prev_state is not None:
            self.sync_cosim_actor_to_carla(self._direct_prev_state)
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla(self._direct_prev_state)

        self.world.tick()
        commands, feedback_records = self._collect_ackermann_feedback()
        try:
            self._direct_tick_future = self.direct_link.tick_async(commands)
        except Exception as exc:
            self._finalize_ackermann_feedback_records(
                feedback_records,
                accepted=False,
                reason=f"feedback_transport_error:{type(exc).__name__}",
            )
            return False
        self._finalize_ackermann_feedback_records(
            feedback_records,
            accepted=True,
            reason="accepted_by_grpc_tick",
        )
        return True

    def tick(self):
        if self.direct_link is not None:
            return self._tick_direct()
        if self.ackermann_feedback_apply_enabled:
            return self._tick_ackermann_feedback_apply_http()
        if self.async_mode:
            time_start = time.time()
            if self.control_av:
                self.sync_carla_av_to_cosim()

            self.sync_cosim_actor_to_carla()
            self.sync_cosim_tls_to_carla()

            self.world.tick()
            if self.ackermann_feedback_shadow_enabled:
                self.sync_carla_ackermann_feedback_to_cosim()
            time_end = time.time()
            elapsed = time_end - time_start
            if elapsed < self.step_length:
                time.sleep(self.step_length - elapsed)
        else:
            while True:
                terasim_status_http_response = get_terasim_status(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])
                terasim_status = terasim_status_http_response.get("status", None)
                if terasim_status == "ticked" or terasim_status == "wait_for_tick":
                    break
                elif terasim_status is None:
                    print("TeraSim status is None. Exiting...")
                    return False
                else:
                    time.sleep(0.05)

            if self.control_av:
                self.sync_carla_av_to_cosim()

            self.sync_cosim_actor_to_carla()
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla()

            tick_terasim(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

            # 3-cosim passive mode: the psim bridge (autoware_carla_interface) is the sole
            # owner of world.tick(). CarlaCosim does not tick the world; it waits for the
            # psim tick so the two clients stay synchronized on one CARLA server.
            if getattr(self.args, "passive_tick", False):
                self.world.wait_for_tick()
            else:
                self.world.tick()
            if self.ackermann_feedback_shadow_enabled:
                self.sync_carla_ackermann_feedback_to_cosim()
        return True

    def _tick_direct(self):
        """One co-sim step over the direct gRPC link (no Redis/HTTP/polling).

        Pipeline parity with the polling path: the state rendered into CARLA is
        the previous step's state, and the SUMO step requested this tick
        computes in the background while this client waits for the CARLA tick.
        """
        if self.ackermann_feedback_apply_enabled:
            return self._tick_ackermann_feedback_apply_direct()

        import grpc as _grpc

        from .direct_link import parse_state_json

        # Resolve the step requested on the previous tick.
        if self._direct_tick_future is not None:
            try:
                resp = self._direct_tick_future.result(timeout=300.0)
            except _grpc.RpcError as e:
                print(f"TeraSim direct tick failed: {e}. Exiting...")
                return False
            if resp.status in ("finished", "error"):
                print(f"TeraSim ended (status={resp.status}). Exiting...")
                return False
            state = parse_state_json(resp.state_json)
            if state is not None:
                self._direct_prev_state = state

        commands = []
        if self.control_av:
            av_command = self._build_av_command()
            if av_command is not None:
                commands.append(av_command)

        # Request the next SUMO step; it runs while CARLA ticks below.
        self._direct_tick_future = self.direct_link.tick_async(commands)

        # Render the previous step's state (same one-step latency as the
        # polling path, which renders the state before requesting its tick).
        if self._direct_prev_state is not None:
            self.sync_cosim_actor_to_carla(self._direct_prev_state)
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla(self._direct_prev_state)

        if getattr(self.args, "passive_tick", False):
            self.world.wait_for_tick()
        else:
            self.world.tick()
        if self.ackermann_feedback_shadow_enabled:
            self.sync_carla_ackermann_feedback_to_cosim()
        return True

    def sync_carla_av_to_cosim(self):
        """Build the AV set_state command and send it over the HTTP service link."""
        av_command = self._build_av_command()
        if av_command is None:
            return
        control_agent(
            self.args.terasim_host,
            self.args.terasim_port,
            self.terasim["simulation_id"],
            av_command,
        )

    def _build_av_command(self):
        # 3-cosim: the ego that drives in CARLA is the Autoware ego (role "ego_vehicle"), not the
        # SUMO-spawned "AV". Read that actor's pose and build a set_state command for the SUMO AV
        # so background traffic avoids it. Returns None when the command cannot be built yet
        # (actor missing / av_shape not initialized). Sending is up to the caller: the polling
        # path POSTs it (sync_carla_av_to_cosim), the direct path attaches it to the Tick RPC.
        av_role = getattr(self.args, "av_carla_role", AV_SUMO_ID)
        vehicle_status, carla_id = get_actor_id_from_attribute(self.world, av_role)

        if not vehicle_status:
            print(f"AV source actor (role={av_role}) not found in Carla simulation.")
            return None

        if not self.av_shape:
            # av_shape is filled by initialize_av in sync_cosim_actor_to_carla, which runs later in
            # the same tick. Skip until then to avoid indexing an empty shape on the first tick.
            return None

        AV = self.world.get_actor(carla_id)
        if AV is None:
            print(f"AV actor {carla_id} not resolvable this loop; command skipped.", flush=True)
            return None
        transform = AV.get_transform()
        draw_text(self.world, transform.location + carla.Location(z=2.5), AV_SUMO_ID)
        # draw_point(
        #     self.world,
        #     size=0.05,
        #     color=(255, 0, 0),
        #     location=transform.location + carla.Location(z=2.5),
        #     life_time=0,
        # )

        velocity = AV.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

        # Reverse transform: CARLA -> SUMO
        if self._coord_transformer is not None:
            # Direct reverse: CARLA location -> xodr coords -> UTM -> SUMO CRS
            # CARLA: x = xodr_x, y = -xodr_y
            xodr_x = transform.location.x
            xodr_y = -transform.location.y
            sumo_x, sumo_y = self._transform_xodr_to_sumo(xodr_x, xodr_y)
            # Apply vehicle shape correction (SUMO position is front bumper)
            yaw = math.radians(90.0 - (-1 * transform.rotation.yaw + 90))
            sumo_x += math.cos(yaw) * self.av_shape[0] / 2.0
            sumo_y += math.sin(yaw) * self.av_shape[0] / 2.0
            av_sumo_location = [sumo_x, sumo_y, transform.location.z]
            av_sumo_rotation = [transform.rotation.pitch, transform.rotation.yaw + 90, transform.rotation.roll]
        else:
            av_offset = [self.sumo_carla_offset[0], self.sumo_carla_offset[1], 0.0]
            av_sumo_location, av_sumo_rotation = carla_to_sumo(
                transform.location,
                transform.rotation,
                self.av_shape,
                av_offset
            )

        av_command = {
            "agent_id": AV_SUMO_ID,
            "agent_type": "vehicle",
            "command_type": "set_state",
            "data": {
                "position": [av_sumo_location[0], av_sumo_location[1]],
                "speed": speed,
                "sumo_angle": av_sumo_rotation[1],
            }
        }

        return av_command

    def _get_ackermann_feedback_shape(self, actor_id):
        vehicle_states = (
            self.terasim_states.get("agent_details", {}).get("vehicle", {})
            if isinstance(self.terasim_states, dict)
            else {}
        )
        vehicle_state = vehicle_states.get(actor_id)
        if not isinstance(vehicle_state, dict):
            return None
        shape = [
            self._as_finite_float(vehicle_state.get("length")),
            self._as_finite_float(vehicle_state.get("width")),
            self._as_finite_float(vehicle_state.get("height")),
        ]
        if any(value is None or value <= 0.0 for value in shape):
            return None
        return shape

    def _carla_transform_to_sumo_feedback_state(self, transform, shape):
        location = transform.location
        rotation = transform.rotation
        values = [
            self._as_finite_float(location.x),
            self._as_finite_float(location.y),
            self._as_finite_float(location.z),
            self._as_finite_float(rotation.yaw),
        ]
        if any(value is None for value in values):
            return None

        carla_x, carla_y, carla_z, carla_yaw = values
        if self._coord_transformer is not None:
            yaw = math.radians(carla_yaw)
            front_xodr_x = carla_x + math.cos(yaw) * shape[0] / 2.0
            front_xodr_y = -carla_y - math.sin(yaw) * shape[0] / 2.0
            sumo_x, sumo_y = self._transform_xodr_to_sumo(front_xodr_x, front_xodr_y)
            sumo_location = [sumo_x, sumo_y, carla_z]
            sumo_rotation = [rotation.pitch, carla_yaw + 90.0, rotation.roll]
        else:
            offset = [self.sumo_carla_offset[0], self.sumo_carla_offset[1], 0.0]
            sumo_location, sumo_rotation = carla_to_sumo(
                location, rotation, shape, offset
            )

        sumo_values = [
            self._as_finite_float(sumo_location[0]),
            self._as_finite_float(sumo_location[1]),
            self._as_finite_float(sumo_rotation[1]),
        ]
        if any(value is None for value in sumo_values):
            return None
        return {
            "position": [sumo_values[0], sumo_values[1]],
            "sumo_angle": sumo_values[2] % 360.0,
        }

    def _is_ackermann_feedback_selected_actor(self, actor_id):
        return actor_id in self.ackermann_feedback_actor_ids or (
            actor_id != AV_SUMO_ID and self.ackermann_feedback_all_background_actors
        )

    def _new_ackermann_feedback_record(self, actor_id):
        return {
            "simulation_time": self.terasim_states.get("simulation_time", ""),
            "actor_id": actor_id,
            "feedback_mode": self.ackermann_feedback_mode,
            "feedback_status": "rejected",
        }

    def _prepare_ackermann_feedback(self, actor_id, actor, snapshot):
        feedback = self._new_ackermann_feedback_record(actor_id)
        shape = self._get_ackermann_feedback_shape(actor_id)
        if shape is None:
            feedback["feedback_reason"] = "sumo_shape_missing_or_invalid"
            return None, feedback
        if actor is None:
            feedback["feedback_reason"] = "carla_actor_missing"
            return None, feedback

        try:
            transform = actor.get_transform()
            velocity = actor.get_velocity()
        except Exception as exc:
            feedback["feedback_reason"] = f"carla_state_error:{type(exc).__name__}"
            return None, feedback

        carla_frame = int(snapshot.frame)
        previous_frame = self._ackermann_feedback_state.get(actor_id, {}).get("source_carla_frame")
        if previous_frame is not None and carla_frame <= previous_frame:
            feedback["source_carla_frame"] = carla_frame
            feedback["feedback_reason"] = "stale_or_duplicate_carla_frame"
            return None, feedback

        sumo_state = self._carla_transform_to_sumo_feedback_state(transform, shape)
        speed = self._as_finite_float(horizontal_speed(velocity))
        if sumo_state is None or speed is None:
            feedback["source_carla_frame"] = carla_frame
            feedback["feedback_reason"] = "non_finite_feedback_state"
            return None, feedback

        command = {
            "agent_id": actor_id,
            "agent_type": "vehicle",
            "command_type": "set_state",
            "data": {
                "position": sumo_state["position"],
                "speed": speed,
                "sumo_angle": sumo_state["sumo_angle"],
                "source_carla_frame": carla_frame,
            },
        }
        feedback.update(
            {
                "source_carla_frame": carla_frame,
                "carla_speed": speed,
                "feedback_sumo_x": sumo_state["position"][0],
                "feedback_sumo_y": sumo_state["position"][1],
                "feedback_sumo_angle": sumo_state["sumo_angle"],
            }
        )
        return command, feedback

    def _record_ackermann_feedback(self, feedback):
        self._ackermann_feedback_state[feedback["actor_id"]] = feedback
        if getattr(self, "ackermann_feedback_log_records", True):
            print("AckermannFeedback " + json.dumps(feedback, sort_keys=True), flush=True)

    def _collect_ackermann_feedback(self):
        candidate_actor_ids = set(self._ackermann_feedback_candidate_actor_ids)
        if not candidate_actor_ids:
            vehicle_states = self.terasim_states.get("agent_details", {}).get("vehicle", {})
            candidate_actor_ids = {
                actor_id
                for actor_id in vehicle_states
                if self._is_ackermann_feedback_selected_actor(actor_id)
            }
        if not candidate_actor_ids:
            return [], []

        try:
            snapshot = self.world.get_snapshot()
        except Exception as exc:
            records = []
            for actor_id in sorted(candidate_actor_ids):
                feedback = self._new_ackermann_feedback_record(actor_id)
                feedback["feedback_reason"] = f"carla_snapshot_error:{type(exc).__name__}"
                records.append(feedback)
            return [], records

        commands = []
        records = []
        for actor_id in sorted(candidate_actor_ids):
            actor = self._ackermann_feedback_actor_index.get(actor_id)
            if actor is None:
                found, carla_id = get_actor_id_from_attribute(self.world, actor_id)
                actor = self.world.get_actor(carla_id) if found else None
            command, feedback = self._prepare_ackermann_feedback(
                actor_id, actor, snapshot
            )
            records.append(feedback)
            if command is not None:
                commands.append(command)
        return commands, records

    def _finalize_ackermann_feedback_records(
        self, records, *, accepted, reason, accepted_status="queued"
    ):
        for feedback in records:
            if "source_carla_frame" in feedback and "feedback_reason" not in feedback:
                feedback["feedback_status"] = accepted_status if accepted else "rejected"
                feedback["feedback_reason"] = reason
            self._record_ackermann_feedback(feedback)
        return bool(records) and all(
            feedback["feedback_status"] in {"queued", "shadow"} for feedback in records
        )

    def sync_carla_ackermann_feedback_to_cosim(self):
        commands, records = self._collect_ackermann_feedback()
        if self.ackermann_feedback_shadow_enabled:
            return self._finalize_ackermann_feedback_records(
                records,
                accepted=True,
                reason="not_applied",
                accepted_status="shadow",
            )

        accepted = False
        reason = "batch_command_not_queued"
        if commands:
            try:
                response = control_agents_batch(
                    self.args.terasim_host,
                    self.args.terasim_port,
                    self.terasim["simulation_id"],
                    commands,
                )
                queued_count = response.get("queued_count") if isinstance(response, dict) else None
                accepted = queued_count == len(commands)
                reason = "accepted_by_http_batch_queue" if accepted else reason
            except Exception as exc:
                reason = f"feedback_transport_error:{type(exc).__name__}"
        return self._finalize_ackermann_feedback_records(
            records, accepted=accepted, reason=reason
        )

    def sync_cosim_tls_to_carla(self, terasim_states=None):
        if terasim_states is None:
            terasim_states = get_terasim_states(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

        if not terasim_states:
            print("terasim_states not available.")
            return
        
        if "traffic_light_details" not in terasim_states:
            print("No traffic light details available.")
            return

        terasim_tls_data = terasim_states["traffic_light_details"]

        for node_id, node_info in terasim_tls_data.items():
            sumo_tls = node_info["tls"]
            sumo_information = json.loads(node_info["information"])
            parameters = None
            for program_id, program in sumo_information["programs"].items():
                try:
                    parameters = program["parameters"]                
                    break
                except KeyError:
                    print(f"KeyError: Node ({node_id}) Program ({program}) does not have 'parameters' key.")
                    continue
            if parameters is None:
                print(f"Traffic Lights within Node ({node_id}) is not synchronized with Carla.")
                continue
            
            for i in range(len(sumo_tls)):
                param_key = f"{SUMO_CARLA_TLS_LINK_PREFIX}{i}"
                carla_landmark_ids = parameters.get(param_key, "")
                if carla_landmark_ids == "":
                    continue
                carla_landmark_ids = carla_landmark_ids.split(" ")
                for landmark_id in carla_landmark_ids:
                    light_id = int(landmark_id)
                    light_actor = self.world.get_actor(light_id)
                    if not light_actor:
                        print(f"Traffic light with ID {light_id} not found in CARLA.")
                        continue

                    # Defensive guard: CARLA may return a non-TrafficLight Actor
                    # when SUMO's TLS program parameters are not mapped to a
                    # real CARLA landmark_id (e.g. netconvert --tls.guess nets
                    # like Town01). Calling set_state on such an actor raises
                    # AttributeError and aborts the whole cosim tick.
                    if not isinstance(light_actor, carla.TrafficLight):
                        continue

                    light_state = sumo_tls[i]
                    if light_state == "G" or light_state == "g":
                        light_actor.set_state(carla.TrafficLightState.Green)
                    elif light_state == "Y" or light_state == "y":
                        light_actor.set_state(carla.TrafficLightState.Yellow)
                    elif light_state == "R" or light_state == "r":
                        light_actor.set_state(carla.TrafficLightState.Red)

    @staticmethod
    def _actor_xy(actor_info):
        try:
            return float(actor_info["x"]), float(actor_info["y"])
        except (KeyError, TypeError, ValueError):
            return None

    def _filter_actor_details_by_radius(self, vehicles, vrus):
        if not self.actor_filter_enabled:
            return vehicles, vrus

        center_info = vehicles.get(self.actor_filter_center_id)
        center_xy = self._actor_xy(center_info) if center_info is not None else None
        if center_xy is None:
            if not self._actor_filter_missing_center_warned:
                print(
                    "Warning: CARLA co-sim actor radius filter center "
                    f"{self.actor_filter_center_id!r} is not available; "
                    "using unfiltered actors.",
                    flush=True,
                )
                self._actor_filter_missing_center_warned = True
            return vehicles, vrus

        center_x, center_y = center_xy
        radius_squared = self.actor_filter_radius * self.actor_filter_radius

        filtered_vehicles = {}
        for veh_id, veh_info in vehicles.items():
            if veh_id in {self.actor_filter_center_id, AV_SUMO_ID}:
                filtered_vehicles[veh_id] = veh_info
                continue
            vehicle_xy = self._actor_xy(veh_info)
            if vehicle_xy is None:
                filtered_vehicles[veh_id] = veh_info
                continue
            dx = vehicle_xy[0] - center_x
            dy = vehicle_xy[1] - center_y
            if dx * dx + dy * dy <= radius_squared:
                filtered_vehicles[veh_id] = veh_info

        return filtered_vehicles, vrus

    def sync_cosim_actor_to_carla(self, terasim_states=None):
        """Update all actors in cosim to CARLA.

        terasim_states: pass the state dict directly (direct/gRPC mode); None
        fetches it from the service over HTTP (Redis-era path).
        """
        if terasim_states is None:
            terasim_states = get_terasim_states(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])
        self.terasim_states = terasim_states or {}

        if not terasim_states:
            print("terasim_states not available.")
            return
        
        if "agent_details" not in terasim_states:
            print("No agent details available.")
            return
        
        if "vehicle" not in terasim_states["agent_details"]:
            print("No vehicle details available.")
            return
        
        if "vru" not in terasim_states["agent_details"]:
            print("No VRU details available.")
            return

        cosim_id_record = set()
        current_frame = self.world.get_snapshot().frame
        vehicle_actor_index, pedestrian_actor_index = self._build_actor_role_indexes()
        vehicles = terasim_states["agent_details"]["vehicle"]
        vrus = terasim_states["agent_details"]["vru"]
        vehicles, vrus = self._filter_actor_details_by_radius(vehicles, vrus)
        feedback_candidate_actor_ids = {
            veh_id
            for veh_id in vehicles
            if self._is_ackermann_feedback_selected_actor(veh_id)
        }
        transform_batch = []
        ackermann_batch = []
        spawn_requests = []

        for veh_id, veh_info in vehicles.items():
            if self.control_av and veh_id == AV_SUMO_ID:
                if not self.initialize_av:
                    self.initialize_av = True
                    self.av_shape = [
                        veh_info["length"],
                        veh_info["width"],
                        veh_info["height"],
                    ]
                    print("AV is initialized based on SUMO state.")
                # In control_av / 3-cosim mode, the CARLA-side ego actor already
                # represents the AV. Do not spawn a second SUMO AV into CARLA.
                continue

            self._process_vehicle(
                veh_id,
                veh_info,
                cosim_id_record,
                carla_actor=vehicle_actor_index.get(veh_id),
                actor_index=vehicle_actor_index,
                current_frame=current_frame,
                transform_batch=transform_batch,
                ackermann_batch=ackermann_batch,
                spawn_requests=spawn_requests,
            )
        
        for vru_id, vru_info in vrus.items():
            vru_actor_index = (
                vehicle_actor_index
                if self._vru_uses_vehicle_blueprint(vru_info)
                else pedestrian_actor_index
            )
            self._process_vru(
                vru_id,
                vru_info,
                cosim_id_record,
                carla_actor=vru_actor_index.get(vru_id),
                actor_index=vru_actor_index,
                current_frame=current_frame,
                transform_batch=transform_batch,
                spawn_requests=spawn_requests,
            )

        self._flush_actor_spawn_batch(spawn_requests, transform_batch)
        self._flush_actor_transform_batch(transform_batch)
        self._flush_actor_ackermann_batch(ackermann_batch)
        self._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
        self._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)
        self._ackermann_feedback_candidate_actor_ids = feedback_candidate_actor_ids
        self._ackermann_feedback_actor_index = {
            actor_id: vehicle_actor_index[actor_id]
            for actor_id in feedback_candidate_actor_ids
            if actor_id in vehicle_actor_index
        }
        self._prune_spawn_failures(vehicles.keys(), vrus.keys())
        self._prune_ackermann_actor_state(vehicles.keys())
        self._prune_ackermann_feedback_state(feedback_candidate_actor_ids)

        # self.sync_cosim_tls_to_carla()

    def sync_cosim_construction_zone_to_carla(self):
        def add_interpolated_points(points, offset):
            """
            Interpolates additional points to ensure no two consecutive points
            after UTM transformation have a distance greater than the specified offset.
            """
            refined_points = []
            print("enter add_interpolated_points")
            for i in range(len(points) - 1):
                p1 = points[i]
                p2 = points[i + 1]
                # p1 = utm_to_carla(points[i][0], points[i][1])
                # p2 = utm_to_carla(points[i + 1][0], points[i + 1][1])
                refined_points.append(p1)  # Add the current transformed point

                # Calculate the 2D distance between transformed points (x, y only)
                distance = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
                if distance > offset:
                    # Add intermediate points
                    num_new_points = int(distance // offset)
                    for j in range(1, num_new_points + 1):
                        # Linear interpolation to find new points
                        new_x = p1[0] + j * (p2[0] - p1[0]) / (num_new_points + 1)
                        new_y = p1[1] + j * (p2[1] - p1[1]) / (num_new_points + 1)
                        refined_points.append((new_x, new_y))

            refined_points.append(points[-1])  # Add the last transformed point
            return refined_points

        try:
            construction_zone_info = self.redis_client.get(CONSTRUCTION_ZONE_INFO)
            if not construction_zone_info:
                print("construction_zone_info is None or empty")
                return
        except Exception as e:
            print(f"Error fetching construction zone info: {e}")
            return

        print("entering construction zone")
        if construction_zone_info:
            closed_lane_shapes = construction_zone_info.closed_lane_shapes

            for closed_lane_shape in closed_lane_shapes:
                closed_lane_shape = add_interpolated_points(closed_lane_shape, 10)
                for cone_point in closed_lane_shape:
                    construction_cone = create_construction_zone_blueprint(self.world)
                    spawn_point = carla.Transform()
                    spawn_point.location.x, spawn_point.location.y = utm_to_carla(
                        cone_point[0], cone_point[1]
                    )
                    spawn_point.location.z = get_z_offset(
                        self.world,
                        start_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 300
                        ),
                        end_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 200
                        ),
                    )
                    id = spawn_actor(
                        client=self.client,
                        blueprint=construction_cone,
                        transform=spawn_point,
                        world=self.world,
                        actor_role="construction_cone",
                    )
                    print(f"created construction cone: {id}")

    def _transform_sumo_to_xodr(self, sumo_x, sumo_y):
        """Transform SUMO coordinates to xodr/CARLA coordinate system.

        Steps: SUMO internal coords -> raw CRS coords -> standard UTM -> subtract xodr origin.
        Returns (xodr_x, xodr_y) where xodr_y still needs to be negated for CARLA.
        """
        if self._coord_transformer is not None:
            # SUMO internal coords = raw coords + netOffset
            raw_x = sumo_x - self._sumo_net_offset[0]
            raw_y = sumo_y - self._sumo_net_offset[1]
            # Transform to standard UTM
            utm_x, utm_y = self._coord_transformer.transform(raw_x, raw_y)
            # Subtract xodr origin to get xodr-local coordinates
            xodr_x = utm_x - self._xodr_origin_utm[0]
            xodr_y = utm_y - self._xodr_origin_utm[1]
            return xodr_x, xodr_y
        return None, None

    def _transform_xodr_to_sumo(self, xodr_x, xodr_y):
        """Reverse transform: xodr/CARLA coordinates -> SUMO coordinates.

        Steps: add xodr origin -> standard UTM -> SUMO CRS -> add netOffset.
        """
        if self._coord_transformer is not None:
            # xodr-local -> standard UTM
            utm_x = xodr_x + self._xodr_origin_utm[0]
            utm_y = xodr_y + self._xodr_origin_utm[1]
            # Reverse transform: UTM -> SUMO CRS
            # _coord_transformer goes SUMO CRS -> UTM, we need the inverse
            raw_x, raw_y = self._coord_transformer.transform(utm_x, utm_y, direction='INVERSE')
            # Add netOffset
            sumo_x = raw_x + self._sumo_net_offset[0]
            sumo_y = raw_y + self._sumo_net_offset[1]
            return sumo_x, sumo_y
        return None, None

    def _get_carla_offset(self, sumo_location, z_offset):
        """Get the offset for sumo_to_carla, incorporating coordinate transformation.
        If using projection-based transform, converts SUMO coords to xodr coords and
        computes the effective offset. Otherwise, returns the calibrated static offset.
        """
        if self._coord_transformer is not None:
            xodr_x, xodr_y = self._transform_sumo_to_xodr(sumo_location[0], sumo_location[1])
            # sumo_to_carla computes: carla_x = sumo_x - cos*shape/2 + offset_x
            #                          carla_y = -(sumo_y - sin*shape/2) + offset_y
            # We want: carla_x ≈ xodr_x, carla_y ≈ -xodr_y
            # So: offset_x = xodr_x - sumo_x (approximately, ignoring shape term)
            #     offset_y = -xodr_y - (-sumo_y) = sumo_y - xodr_y
            return [xodr_x - sumo_location[0], sumo_location[1] - xodr_y, z_offset]
        return [self.sumo_carla_offset[0], self.sumo_carla_offset[1], z_offset]

    def _sumo_point_to_carla_location(self, sumo_location, z_offset=0.0):
        return sumo_point_to_carla(
            sumo_location, self._get_carla_offset(sumo_location, z_offset)
        )

    def _resolve_sumo_lookahead_location(self, veh_id, veh_info, sumo_location, sumo_angle):
        if bool(veh_info.get("lookahead_position_valid", False)):
            lookahead = [
                self._as_finite_float(veh_info.get("lookahead_x")),
                self._as_finite_float(veh_info.get("lookahead_y")),
                self._as_finite_float(veh_info.get("lookahead_z")),
            ]
            if lookahead[2] is None:
                lookahead[2] = sumo_location[2]
            if lookahead[0] is not None and lookahead[1] is not None:
                return lookahead

        desired_speed = self._resolve_ackermann_desired_speed(veh_id, veh_info)
        lookahead_distance = min(15.0, max(7.0, desired_speed))
        heading = math.radians(90.0 - sumo_angle)
        return [
            sumo_location[0] + math.cos(heading) * lookahead_distance,
            sumo_location[1] + math.sin(heading) * lookahead_distance,
            sumo_location[2],
        ]

    @staticmethod
    def _set_actor_simulate_physics(actor, enabled):
        try:
            actor.set_simulate_physics(enabled)
        except Exception:
            pass

    def _ensure_ackermann_actor_physics(self, actor, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if state.get("physics_enabled"):
            return
        self._set_actor_simulate_physics(actor, True)
        state["physics_enabled"] = True

    def _ensure_actor_teleport_mode(self, actor, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if state.get("physics_enabled") is False:
            return
        self._set_actor_simulate_physics(actor, False)
        state.clear()
        state["physics_enabled"] = False

    def _warn_ackermann_position_error(self, veh_id, position_error):
        if self.ackermann_warn_error_m <= 0.0 or position_error < self.ackermann_warn_error_m:
            return
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        now = time.monotonic()
        last_warning_time = state.get("last_warning_time", 0.0)
        if now - last_warning_time < self.ackermann_warning_interval:
            return
        state["last_warning_time"] = now
        print(
            f"Warning: Ackermann vehicle {veh_id!r} is {position_error:.2f}m "
            "from its SUMO desired position.",
            flush=True,
        )

    def _is_ackermann_feedback_apply_actor(self, veh_id):
        return self.ackermann_feedback_apply_enabled and (
            self._is_ackermann_feedback_selected_actor(veh_id)
        )

    def _uses_ackermann_physics(self, veh_id):
        if not self.ackermann_physics_enabled:
            return False
        if self.ackermann_feedback_apply_enabled:
            return self._is_ackermann_feedback_apply_actor(veh_id)
        return True

    def _is_ackermann_feedback_healthy(self, veh_id, feedback, observed_frame):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if feedback.get("feedback_status") != "queued":
            state["feedback_ack_failures"] = 0
            return False

        expected_frame = self._as_finite_float(feedback.get("source_carla_frame"))
        observed_frame = self._as_finite_float(observed_frame)
        frame_lag = (
            expected_frame - observed_frame
            if expected_frame is not None and observed_frame is not None
            else None
        )
        ack_missing = frame_lag is None or frame_lag > self.ackermann_feedback_ack_max_frame_lag
        failures = state.get("feedback_ack_failures", 0) + 1 if ack_missing else 0
        state["feedback_ack_failures"] = failures
        state["feedback_frame_lag"] = frame_lag
        return failures < self.ackermann_feedback_ack_failure_limit

    def _resolve_ackermann_desired_speed(self, veh_id, veh_info):
        speed_key = (
            "sumo_desired_speed"
            if self._is_ackermann_feedback_apply_actor(veh_id)
            else "speed"
        )
        desired_speed = self._as_finite_float(veh_info.get(speed_key))
        if desired_speed is None and speed_key != "speed":
            desired_speed = self._as_finite_float(veh_info.get("speed"))
        return max(0.0, desired_speed or 0.0)

    def _resolve_ackermann_longitudinal_target(self, veh_id, veh_info, current_speed):
        desired_speed = self._resolve_ackermann_desired_speed(veh_id, veh_info)
        if not self._is_ackermann_feedback_apply_actor(veh_id):
            return desired_speed, None

        sumo_next_speed = self._as_finite_float(veh_info.get("sumo_desired_speed"))
        observed_speed = self._as_finite_float(veh_info.get("feedback_observed_speed"))
        if sumo_next_speed is None or observed_speed is None or self.step_length <= 0.0:
            return desired_speed, None
        desired_acceleration = (sumo_next_speed - observed_speed) / self.step_length
        desired_acceleration = min(
            self.ackermann_tuning.max_accel,
            max(-self.ackermann_tuning.max_decel, desired_acceleration),
        )
        speed_target = max(
            0.0,
            current_speed + desired_acceleration * self.ackermann_feedback_speed_horizon,
        )
        return speed_target, desired_acceleration

    def _neutralize_ackermann_steer(self, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        steer = self._as_finite_float(state.get("steer")) or 0.0
        max_delta = self.ackermann_tuning.max_steer_rate_rad_s * self.step_length
        if max_delta <= 0.0 or abs(steer) <= max_delta:
            return 0.0
        return steer - math.copysign(max_delta, steer)

    def _make_brake_ackermann_control(self, veh_id):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        if self._is_ackermann_feedback_apply_actor(veh_id):
            steer = self._neutralize_ackermann_steer(veh_id)
            state["steer"] = steer
        else:
            steer = self._as_finite_float(state.get("steer")) or 0.0
        return carla.VehicleAckermannControl(
            steer=steer,
            speed=0.0,
            acceleration=-self.ackermann_tuning.max_decel,
            jerk=0.0,
        )

    def _build_ackermann_control(
        self,
        veh_id,
        veh_info,
        vehicle,
        sumo_location,
        sumo_angle,
        desired_transform,
    ):
        state = self._ackermann_actor_state.setdefault(veh_id, {})
        try:
            current_transform = vehicle.get_transform()
            current_velocity = vehicle.get_velocity()
        except Exception:
            return self._make_brake_ackermann_control(veh_id)

        desired_location = desired_transform.location
        current_location = current_transform.location
        position_error = math.hypot(
            current_location.x - desired_location.x,
            current_location.y - desired_location.y,
        )
        self._warn_ackermann_position_error(veh_id, position_error)

        if (
            self.ackermann_snap_error_m > 0.0
            and position_error >= self.ackermann_snap_error_m
            and not state.get("snap_used", False)
            and not self._is_ackermann_feedback_apply_actor(veh_id)
        ):
            try:
                vehicle.set_transform(desired_transform)
                current_transform = desired_transform
                current_location = desired_location
                state["steer"] = 0.0
                state["snap_used"] = True
            except Exception:
                pass

        lookahead_sumo_location = self._resolve_sumo_lookahead_location(
            veh_id, veh_info, sumo_location, sumo_angle
        )
        try:
            lookahead_location = self._sumo_point_to_carla_location(
                lookahead_sumo_location
            )
        except Exception:
            return self._make_brake_ackermann_control(veh_id)

        current_speed = horizontal_speed(current_velocity)
        desired_speed, feedback_desired_acceleration = (
            self._resolve_ackermann_longitudinal_target(veh_id, veh_info, current_speed)
        )
        values = compute_ackermann_control_values(
            current_x=current_location.x,
            current_y=current_location.y,
            yaw_degrees=current_transform.rotation.yaw,
            current_speed=current_speed,
            desired_x=desired_location.x,
            desired_y=desired_location.y,
            lookahead_x=lookahead_location.x,
            lookahead_y=lookahead_location.y,
            desired_speed=desired_speed,
            previous_steer=state.get("steer"),
            dt=self.step_length,
            tuning=self.ackermann_tuning,
        )
        final_steer = values.steer
        final_speed = values.speed
        final_acceleration = (
            values.acceleration
            if feedback_desired_acceleration is None
            else feedback_desired_acceleration
        )
        feedback = self._ackermann_feedback_state.get(veh_id, {})
        feedback_unhealthy = (
            self._is_ackermann_feedback_apply_actor(veh_id)
            and feedback
            and not self._is_ackermann_feedback_healthy(
                veh_id, feedback, veh_info.get("feedback_source_carla_frame")
            )
        )
        target_behind = (
            self._is_ackermann_feedback_apply_actor(veh_id)
            and values.lookahead_local_x <= 0.0
        )
        if feedback_unhealthy or target_behind:
            final_steer = self._neutralize_ackermann_steer(veh_id)
            final_speed = 0.0
            final_acceleration = -self.ackermann_tuning.max_decel

        state["steer"] = final_steer
        state["last_position_error"] = values.position_error
        return carla.VehicleAckermannControl(
            steer=final_steer,
            speed=final_speed,
            acceleration=final_acceleration,
            jerk=values.jerk,
        )

    def _queue_actor_ackermann_control(self, actor, control, ackermann_batch=None):
        apply_command = getattr(carla.command, "ApplyVehicleAckermannControl", None)
        if ackermann_batch is not None and apply_command is not None:
            ackermann_batch.append(apply_command(actor.id, control))
            return
        actor.apply_ackermann_control(control)

    def _flush_actor_ackermann_batch(self, ackermann_batch):
        if not ackermann_batch:
            return []
        return self.client.apply_batch_sync(ackermann_batch, False)

    def _prune_ackermann_actor_state(self, vehicle_ids):
        active_vehicle_ids = set(vehicle_ids)
        for veh_id in list(self._ackermann_actor_state):
            if veh_id not in active_vehicle_ids:
                self._ackermann_actor_state.pop(veh_id, None)

    def _prune_ackermann_feedback_state(self, actor_ids):
        active_actor_ids = set(actor_ids)
        for actor_id in list(self._ackermann_feedback_state):
            if actor_id not in active_actor_ids:
                self._ackermann_feedback_state.pop(actor_id, None)

    # Elevated spawn height to avoid collision with OpenDRIVE-generated road geometry
    # (guardrails, curbs, barriers). After spawn, correct transform is set immediately.
    SPAWN_Z_CLEARANCE = 5.0
    SPAWN_RETRY_FRAMES = 10
    SPAWN_RETRY_DISTANCE = 5.0

    @staticmethod
    def _spawn_failure_key(actor_type, actor_id):
        return actor_type, actor_id

    def _fresh_blueprint(self, blueprint):
        try:
            return self.world.get_blueprint_library().find(blueprint.id)
        except Exception:
            return blueprint

    def _select_vehicle_blueprint(self, veh_id, veh_info):
        if "BIKE" in veh_info["type"]:
            blueprint = random.choice(self.bike_blueprints)
        elif "MOTOR" in veh_info["type"]:
            blueprint = random.choice(self.motor_blueprints)
        elif "POLICE" in veh_info["type"]:
            blueprint = random.choice(self.police_car_blueprints)
        else:
            blueprint = random.choice(self.vehicle_blueprints)
        blueprint = self._fresh_blueprint(blueprint)
        blueprint.set_attribute("role_name", veh_id)
        if veh_id == AV_SUMO_ID:
            blueprint.set_attribute("color", "255, 0, 0")
        else:
            blueprint.set_attribute("color", "0, 102, 204")
        return blueprint

    def _select_vru_blueprint(self, vru_id, vru_info):
        if "BIKE" in vru_info["type"]:
            blueprint = random.choice(self.bike_blueprints)
        elif "MOTOR" in vru_info["type"]:
            blueprint = random.choice(self.motor_blueprints)
        else:
            blueprint = random.choice(self.pedestrian_blueprints)
        blueprint = self._fresh_blueprint(blueprint)
        blueprint.set_attribute("role_name", vru_id)
        return blueprint

    @staticmethod
    def _empty_spawn_batch_stats():
        return {
            "spawn_calls": 0,
            "spawn_total": 0.0,
            "spawn_max": 0.0,
            "spawn_success": 0,
            "spawn_failed": 0,
            "apply_batch_sync_time": 0.0,
            "apply_batch_sync_max": 0.0,
            "apply_batch_sync_success_time": 0.0,
            "apply_batch_sync_failed_time": 0.0,
        }

    def _queue_actor_spawn(self, spawn_requests, request):
        if self.batch_spawn_enabled and spawn_requests is not None:
            spawn_requests.append(request)
            return True
        return False

    def _flush_actor_spawn_batch(self, spawn_requests, transform_batch=None):
        stats = self._empty_spawn_batch_stats()
        if not spawn_requests:
            return stats

        commands = [
            carla.command.SpawnActor(request["blueprint"], request["spawn_transform"]).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False)
            )
            for request in spawn_requests
        ]

        total_start = time.perf_counter()
        apply_start = time.perf_counter()
        # due_tick_cue=False: with True, in synchronous mode this call blocks until the
        # next world tick. Spawns happen every time a vehicle enters the sync radius, so
        # the blocking variant locks the whole co-sim loop to 2 CARLA ticks per step
        # (measured: client wait p50 ~70ms vs the 51ms tick period). The responses
        # (actor ids) are still returned without the tick; an actor that is not yet
        # resolvable falls into the existing spawn-failure retry path.
        responses = self.client.apply_batch_sync(commands, False)
        apply_elapsed = time.perf_counter() - apply_start

        stats["spawn_calls"] = len(spawn_requests)
        stats["apply_batch_sync_time"] = apply_elapsed
        stats["apply_batch_sync_max"] = apply_elapsed

        response_count = len(responses)
        success_count = sum(1 for response in responses if not response.error)
        failed_count = response_count - success_count
        stats["spawn_success"] = success_count
        stats["spawn_failed"] = failed_count
        if response_count:
            stats["apply_batch_sync_success_time"] = apply_elapsed * success_count / response_count
            stats["apply_batch_sync_failed_time"] = apply_elapsed * failed_count / response_count

        for request, response in zip(spawn_requests, responses):
            actor_type = request["actor_type"]
            actor_id = request["actor_id"]
            if response.error:
                log_spawn_actor_failure(
                    self.world,
                    request["blueprint"],
                    request["spawn_transform"],
                    actor_id,
                    response.error,
                )
                self._record_spawn_failure(
                    actor_type, actor_id, request["sumo_location"], request["current_frame"]
                )
                continue

            actor = self.world.get_actor(response.actor_id)
            if actor is None:
                self._record_spawn_failure(
                    actor_type, actor_id, request["sumo_location"], request["current_frame"]
                )
                continue

            self._clear_spawn_failure(actor_type, actor_id)
            actor_index = request.get("actor_index")
            if actor_index is not None:
                actor_index[actor_id] = actor
            if request.get("enable_physics_after_spawn", False):
                self._ackermann_actor_state.pop(actor_id, None)
                actor.set_transform(request["post_spawn_transform"])
                self._ensure_ackermann_actor_physics(actor, actor_id)
            else:
                self._queue_actor_transform(actor, request["post_spawn_transform"], transform_batch)
            if actor_type == "vru":
                self._apply_vru_walker_control(request["actor_info"], request["sumo_angle"], actor)

        stats["spawn_total"] = time.perf_counter() - total_start
        stats["spawn_max"] = stats["spawn_total"]
        return stats

    def _apply_vru_walker_control(self, vru_info, sumo_angle, pedestrian):
        if self._vru_uses_vehicle_blueprint(vru_info):
            return
        radians = math.radians(90 - sumo_angle)
        orientation = math.atan2(math.sin(radians), math.cos(radians))
        direction_x, direction_y = math.cos(orientation), math.sin(orientation)
        walker_control = carla.WalkerControl(
            direction=carla.Vector3D(direction_x, direction_y, 0),
            speed=vru_info["speed"],
        )
        try:
            pedestrian.apply_control(walker_control)
        except Exception:
            pass

    def _queue_actor_transform(self, actor, transform, transform_batch=None):
        """Queue or apply a CARLA actor transform according to batching settings."""
        if self.batch_transform_enabled and transform_batch is not None:
            transform_batch.append(carla.command.ApplyTransform(actor.id, transform))
            return
        actor.set_transform(transform)

    def _flush_actor_transform_batch(self, transform_batch):
        """Apply queued actor transforms in one CARLA batch call."""
        if not transform_batch:
            return []
        return self.client.apply_batch_sync(transform_batch, False)

    def _build_actor_role_indexes(self):
        """Build role_name indexes once per tick instead of scanning CARLA per actor."""
        vehicle_actor_index = {}
        pedestrian_actor_index = {}
        actors = self.world.get_actors()
        world_actor_count = 0
        for actor in actors:
            world_actor_count += 1
            role_name = actor.attributes.get("role_name")
            if not role_name:
                continue
            if actor.type_id.startswith("vehicle."):
                vehicle_actor_index.setdefault(role_name, actor)
            elif actor.type_id.startswith("walker.pedestrian."):
                pedestrian_actor_index.setdefault(role_name, actor)
        self._last_actor_index_world_actor_count = world_actor_count
        self._last_actor_index_vehicle_actor_count = len(vehicle_actor_index)
        self._last_actor_index_pedestrian_actor_count = len(pedestrian_actor_index)
        return vehicle_actor_index, pedestrian_actor_index

    @staticmethod
    def _vru_uses_vehicle_blueprint(vru_info):
        return "BIKE" in vru_info["type"] or "MOTOR" in vru_info["type"]

    def _should_retry_spawn(self, actor_type, actor_id, sumo_location, current_frame):
        if actor_id == AV_SUMO_ID:
            return True

        failure = self._spawn_failures.get(self._spawn_failure_key(actor_type, actor_id))
        if failure is None:
            return True

        dx = sumo_location[0] - failure["x"]
        dy = sumo_location[1] - failure["y"]
        if dx * dx + dy * dy >= self.SPAWN_RETRY_DISTANCE * self.SPAWN_RETRY_DISTANCE:
            return True

        next_retry_time = failure.get("next_retry_time")
        if next_retry_time is not None:
            return time.monotonic() >= next_retry_time
        return current_frame >= failure.get("next_retry_frame", current_frame)

    def _record_spawn_failure(self, actor_type, actor_id, sumo_location, current_frame):
        key = self._spawn_failure_key(actor_type, actor_id)
        previous = self._spawn_failures.get(key, {})
        failures = previous.get("failures", 0) + 1
        exponent = min(failures - 1, 30)
        delay = min(
            self.spawn_failure_backoff_max_seconds,
            self.spawn_failure_backoff_seconds * (2 ** exponent),
        )
        self._spawn_failures[key] = {
            "failures": failures,
            "next_retry_frame": current_frame + self.SPAWN_RETRY_FRAMES,
            "next_retry_time": time.monotonic() + delay,
            "x": sumo_location[0],
            "y": sumo_location[1],
        }

    def _clear_spawn_failure(self, actor_type, actor_id):
        self._spawn_failures.pop(self._spawn_failure_key(actor_type, actor_id), None)

    @staticmethod
    def _as_finite_float(value):
        if isinstance(value, bool):
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if math.isfinite(number):
            return number
        return None

    def _warn_invalid_sumo_location_once(self, actor_type, actor_id, raw_location):
        key = (actor_type, actor_id)
        if key in self._invalid_location_warnings:
            return
        self._invalid_location_warnings.add(key)
        print(
            f"Warning: {actor_type} {actor_id!r} has invalid SUMO location "
            f"{raw_location}; skipping this tick.",
            flush=True,
        )

    def _resolve_sumo_location(self, actor_type, actor_id, actor_info, prefer_lane_relative=False):
        use_reconstructed = (
            prefer_lane_relative
            and bool(actor_info.get("reconstructed_position_valid", False))
        )
        if use_reconstructed:
            raw_location = {
                "x": actor_info.get("reconstructed_x"),
                "y": actor_info.get("reconstructed_y"),
                "z": actor_info.get("reconstructed_z"),
            }
        else:
            raw_location = {
                "x": actor_info.get("x"),
                "y": actor_info.get("y"),
                "z": actor_info.get("z"),
            }
        sumo_location = [
            self._as_finite_float(raw_location["x"]),
            self._as_finite_float(raw_location["y"]),
            self._as_finite_float(raw_location["z"]),
        ]
        if use_reconstructed and any(value is None for value in sumo_location):
            raw_location = {
                "x": actor_info.get("x"),
                "y": actor_info.get("y"),
                "z": actor_info.get("z"),
            }
            sumo_location = [
                self._as_finite_float(raw_location["x"]),
                self._as_finite_float(raw_location["y"]),
                self._as_finite_float(raw_location["z"]),
            ]
        if any(value is None for value in sumo_location):
            self._warn_invalid_sumo_location_once(actor_type, actor_id, raw_location)
            return None
        return sumo_location

    def _warn_missing_sumo_angle_once(self, actor_type, actor_id, action):
        key = (actor_type, actor_id, action)
        if key in self._missing_angle_warnings:
            return
        self._missing_angle_warnings.add(key)
        print(
            f"Warning: {actor_type} {actor_id!r} has missing sumo_angle; {action}.",
            flush=True,
        )

    def _resolve_sumo_angle(self, actor_type, actor_id, actor_info, carla_actor=None):
        angle = self._as_finite_float(actor_info.get("sumo_angle"))
        if angle is not None:
            return angle

        orientation = self._as_finite_float(actor_info.get("orientation"))
        if orientation is not None:
            self._warn_missing_sumo_angle_once(actor_type, actor_id, "using orientation fallback")
            return (90.0 - math.degrees(orientation)) % 360.0

        if carla_actor is not None:
            self._warn_missing_sumo_angle_once(actor_type, actor_id, "using previous CARLA yaw fallback")
            return (carla_actor.get_transform().rotation.yaw + 90.0) % 360.0

        self._warn_missing_sumo_angle_once(actor_type, actor_id, "skipping this tick")
        return None

    def _prune_spawn_failures(self, vehicle_ids, vru_ids):
        active_keys = {
            self._spawn_failure_key("vehicle", vehicle_id) for vehicle_id in vehicle_ids
        }
        active_keys.update(
            self._spawn_failure_key("vru", vru_id) for vru_id in vru_ids
        )
        for key in list(self._spawn_failures):
            if key not in active_keys:
                self._spawn_failures.pop(key, None)

    def _process_vehicle(
        self,
        veh_id,
        veh_info,
        cosim_id_record,
        carla_actor=None,
        actor_index=None,
        current_frame=None,
        transform_batch=None,
        ackermann_batch=None,
        spawn_requests=None,
    ):
        """Process a vehicle actor."""
        cosim_id_record.add(veh_id)

        sumo_location = self._resolve_sumo_location(
            "vehicle",
            veh_id,
            veh_info,
            prefer_lane_relative=self.use_lane_relative_position,
        )
        if sumo_location is None:
            return
        sumo_angle = self._resolve_sumo_angle("vehicle", veh_id, veh_info, carla_actor)
        if sumo_angle is None:
            return
        sumo_rotation = [0.0, sumo_angle, 0.0]
        shape = [veh_info["length"], veh_info["width"], veh_info["height"]]
        uses_ackermann_physics = self._uses_ackermann_physics(veh_id)

        vehicle = carla_actor
        if vehicle is None:
            if current_frame is None:
                current_frame = self.world.get_snapshot().frame
            if not self._should_retry_spawn("vehicle", veh_id, sumo_location, current_frame):
                return
            blueprint = self._select_vehicle_blueprint(veh_id, veh_info)
            # Spawn elevated to avoid collision with road geometry, then set correct transform
            sumo_offset = self._get_carla_offset(sumo_location, self.spawn_z_clearance)
            spawn_transform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            sumo_offset_correct = self._get_carla_offset(sumo_location, 0.0)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset_correct)
            if self._queue_actor_spawn(
                spawn_requests,
                {
                    "actor_type": "vehicle",
                    "actor_id": veh_id,
                    "actor_info": veh_info,
                    "blueprint": blueprint,
                    "spawn_transform": spawn_transform,
                    "post_spawn_transform": carla_trasform,
                    "sumo_location": sumo_location,
                    "current_frame": current_frame,
                    "actor_index": actor_index,
                    "sumo_angle": sumo_angle,
                    "enable_physics_after_spawn": uses_ackermann_physics,
                },
            ):
                return
            carla_id = spawn_actor(
                self.client,
                blueprint,
                spawn_transform,
                world=self.world,
                actor_role=veh_id,
            )
            if carla_id > 0:
                self._clear_spawn_failure("vehicle", veh_id)
                vehicle = self.world.get_actor(carla_id)
                if vehicle is None:
                    self._record_spawn_failure("vehicle", veh_id, sumo_location, current_frame)
                    return
                if actor_index is not None:
                    actor_index[veh_id] = vehicle
                # Immediately set the correct road-level transform
                vehicle.set_transform(carla_trasform)
                if uses_ackermann_physics:
                    self._ackermann_actor_state.pop(veh_id, None)
                    self._ensure_ackermann_actor_physics(vehicle, veh_id)
            else:
                self._record_spawn_failure("vehicle", veh_id, sumo_location, current_frame)
        else:
            sumo_offset = self._get_carla_offset(sumo_location, 0.0)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            if uses_ackermann_physics:
                self._ensure_ackermann_actor_physics(vehicle, veh_id)
                control = self._build_ackermann_control(
                    veh_id,
                    veh_info,
                    vehicle,
                    sumo_location,
                    sumo_angle,
                    carla_trasform,
                )
                self._queue_actor_ackermann_control(vehicle, control, ackermann_batch)
            else:
                self._ensure_actor_teleport_mode(vehicle, veh_id)
                self._queue_actor_transform(vehicle, carla_trasform, transform_batch)

    def _process_vru(
        self,
        vru_id,
        vru_info,
        cosim_id_record,
        carla_actor=None,
        actor_index=None,
        current_frame=None,
        transform_batch=None,
        spawn_requests=None,
    ):
        """Process a pedestrian actor."""
        cosim_id_record.add(vru_id)

        sumo_location = self._resolve_sumo_location("vru", vru_id, vru_info)
        if sumo_location is None:
            return
        sumo_angle = self._resolve_sumo_angle("vru", vru_id, vru_info, carla_actor)
        if sumo_angle is None:
            return
        sumo_rotation = [0.0, sumo_angle, 0.0]
        shape = [vru_info["length"], vru_info["width"], vru_info["height"]]

        pedestrian = carla_actor
        carla_id = pedestrian.id if pedestrian is not None else -1
        if pedestrian is None:
            if current_frame is None:
                current_frame = self.world.get_snapshot().frame
            if not self._should_retry_spawn("vru", vru_id, sumo_location, current_frame):
                return
            blueprint = self._select_vru_blueprint(vru_id, vru_info)
            # Spawn elevated to avoid collision with road geometry
            sumo_offset = self._get_carla_offset(sumo_location, self.spawn_z_clearance)
            spawn_transform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            z_off = 0.0 if "BIKE" in vru_info["type"] else shape[2] / 2.0
            sumo_offset_correct = self._get_carla_offset(sumo_location, z_off)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset_correct)
            if self._queue_actor_spawn(
                spawn_requests,
                {
                    "actor_type": "vru",
                    "actor_id": vru_id,
                    "actor_info": vru_info,
                    "blueprint": blueprint,
                    "spawn_transform": spawn_transform,
                    "post_spawn_transform": carla_trasform,
                    "sumo_location": sumo_location,
                    "current_frame": current_frame,
                    "actor_index": actor_index,
                    "sumo_angle": sumo_angle,
                },
            ):
                return
            carla_id = spawn_actor(
                self.client,
                blueprint,
                spawn_transform,
                world=self.world,
                actor_role=vru_id,
            )
            if carla_id > 0:
                self._clear_spawn_failure("vru", vru_id)
                pedestrian = self.world.get_actor(carla_id)
                if pedestrian is None:
                    self._record_spawn_failure("vru", vru_id, sumo_location, current_frame)
                    return
                if actor_index is not None:
                    actor_index[vru_id] = pedestrian
                pedestrian.set_transform(carla_trasform)
            else:
                self._record_spawn_failure("vru", vru_id, sumo_location, current_frame)
        else:
            z_off = 0.0 if "BIKE" in vru_info["type"] else shape[2] / 2.0
            sumo_offset = self._get_carla_offset(sumo_location, z_off)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
            self._queue_actor_transform(pedestrian, carla_trasform, transform_batch)

        if carla_id > 0:
            self._apply_vru_walker_control(vru_info, sumo_angle, pedestrian)

    def _cleanup_actors(self, actor_type, pattern, cosim_id_record):
        """Clean up CARLA actors not in the cosim actor record."""
        # Protect ego (and any other role names passed via protected_roles). In 3-cosim the
        # psim ego has role_name "ego_vehicle", which must not be destroyed as a "stale" actor.
        protected = getattr(self.args, "protected_roles", None) or ["AV"]
        actors_to_destroy = [
            actor
            for actor in self.world.get_actors().filter(pattern)
            if actor.attributes.get("role_name") not in cosim_id_record
            and actor.attributes.get("role_name") not in protected
        ]

        for actor in actors_to_destroy:
            actor.destroy()

    def close(self):
        """
        Cleans synchronization and resets the simulation settings.
        """
        if not getattr(self.args, "passive_tick", False):
            # Configuring carla simulation in async mode.
            # Skipped in 3-cosim passive mode: the psim bridge owns synchronous_mode, and
            # resetting it here would break psim's sync loop.
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
        
        # Destroy actors. In 3-cosim passive mode, keep ego (protected_roles) and clear only the
        # SUMO-spawned background vehicles/pedestrians; otherwise destroy everything.
        if getattr(self.args, "passive_tick", False):
            protected = getattr(self.args, "protected_roles", None) or ["AV"]
            for actor in self.world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") not in protected:
                    actor.destroy()
            for actor in self.world.get_actors().filter("walker.*"):
                actor.destroy()
        else:
            destroy_all_actors(self.world)

        # stop TeraSim
        if self.direct_link is not None:
            self.direct_link.stop()
            self.direct_link.close()
        else:
            stop_terasim(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

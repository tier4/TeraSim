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
    sumo_to_carla,
    spawn_actor,
)
from ..service import (
    control_agent,
    start_terasim,
    stop_terasim,
    tick_terasim,
    get_terasim_status,
    get_terasim_states,
)

AV_SUMO_ID = "AV"
SUMO_CARLA_TLS_LINK_PREFIX = "linkSignalID:"


def _env_float(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return float(value)
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
        self.inprocess_plugin = getattr(args, "inprocess_plugin", None)
        self._inproc_tick_handle = None
        self._inproc_prev_state = None
        if self.inprocess_plugin is not None:
            # Single-process mode (terasim_service.run_cosim): the TeraSim
            # simulation loop runs on another thread of THIS process; commands
            # and states are exchanged as Python objects (no Redis/HTTP/gRPC,
            # no JSON). The runner already waited for wait_for_tick.
            # Seed the render pipeline with the initial (post-warmup) state so
            # the first tick behaves like the polling path (AV shape init etc.).
            self._inproc_prev_state = self.inprocess_plugin.get_result().state
            self.terasim = {"simulation_id": "inprocess"}
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

    def tick(self):
        if self.inprocess_plugin is not None:
            return self._tick_inprocess()
        if self.async_mode:
            time_start = time.time()
            if self.control_av:
                self.sync_carla_av_to_cosim()

            self.sync_cosim_actor_to_carla()
            self.sync_cosim_tls_to_carla()

            self.world.tick()
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
        return True

    def _tick_inprocess(self):
        """One co-sim step over the in-process link (single process, no RPC).

        Pipeline parity with the two-process paths: the state rendered into
        CARLA is the previous step's state, and the SUMO step requested this
        tick computes on the sim thread while this thread renders into CARLA
        and waits for the CARLA tick.
        """
        # Resolve the step requested on the previous tick.
        if self._inproc_tick_handle is not None:
            try:
                result = self._inproc_tick_handle.result(timeout=300.0)
            except TimeoutError as e:
                print(f"TeraSim in-process tick failed: {e}. Exiting...")
                return False
            if result.status in ("finished", "error"):
                print(f"TeraSim ended (status={result.status}). Exiting...")
                return False
            if result.state is not None:
                self._inproc_prev_state = result.state

        commands = []
        if self.control_av:
            av_command = self._build_av_command()
            if av_command is not None:
                commands.append(av_command)

        # Request the next SUMO step; it runs on the sim thread while this
        # thread renders below and waits for the CARLA tick.
        self._inproc_tick_handle = self.inprocess_plugin.tick_async(commands)

        # Render the previous step's state (same one-step latency as the
        # other paths, which render the state before requesting its tick).
        if self._inproc_prev_state is not None:
            self.sync_cosim_actor_to_carla(self._inproc_prev_state)
            if not getattr(self.args, "skip_tls", False):
                self.sync_cosim_tls_to_carla(self._inproc_prev_state)

        if getattr(self.args, "passive_tick", False):
            self.world.wait_for_tick()
        else:
            self.world.tick()
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
        transform_batch = []
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
        self._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
        self._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)
        self._prune_spawn_failures(vehicles.keys(), vrus.keys())

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
            else:
                self._record_spawn_failure("vehicle", veh_id, sumo_location, current_frame)
        else:
            sumo_offset = self._get_carla_offset(sumo_location, 0.0)
            carla_trasform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
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
        if self.inprocess_plugin is not None:
            self.inprocess_plugin.request_stop()
        else:
            stop_terasim(self.args.terasim_host, self.args.terasim_port, self.terasim["simulation_id"])

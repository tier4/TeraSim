"""CARLA Co-Simulation Client with local-bounds alignment fallback.

This variant keeps the standard CarlaCosim flow, but overrides the
SUMO->CARLA XY offset when the SUMO net.xml has no projection metadata
(`projParameter='!'`) and its metric bounds already match the loaded
CARLA map shape. This is useful for custom packaged maps where both
SUMO and CARLA share the same local metric frame but differ by a fixed
translation and axis flip.
"""
import argparse
import csv
import importlib
import math
import os
import statistics
import time
import xml.etree.ElementTree as ET

import carla
from terasim_service.utils.carla import CarlaCosim
from terasim_service.utils.carla.tools import (
    get_actor_id_from_attribute,
    sumo_to_carla,
)

carla_cosim_module = importlib.import_module("terasim_service.utils.carla.cosim")


def parse_args():
    argparser = argparse.ArgumentParser(
        description="CARLA Co-Simulation Client for TeraSim with local alignment fallback"
    )
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )
    argparser.add_argument(
        "--carla_host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server for Carla (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "--carla_port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to for Carla (default: 2000)",
    )
    argparser.add_argument(
        "-s",
        "--step_length",
        metavar="S",
        default=0.1,
        type=float,
        help="Step length of Carla simulation in seconds (default: 0.1)",
    )
    argparser.add_argument(
        "--control_av",
        action="store_true",
        help="Activate AV manual control mode execution",
    )
    argparser.add_argument(
        "--async_mode",
        action="store_true",
        help="Activate async mode execution",
    )
    argparser.add_argument(
        "--carla_timeout",
        default=10.0,
        type=float,
        help="Timeout in seconds for CARLA client connection (default: 10.0)",
    )
    argparser.add_argument(
        "--map_name",
        default="",
        type=str,
        help="Map name to load (default: empty string)",
    )
    argparser.add_argument(
        "--terasim_host",
        default="localhost",
        help="IP of the host server for TeraSim (default: localhost)",
    )
    argparser.add_argument(
        "--terasim_port",
        default=8000,
        type=int,
        help="TCP port to listen to for TeraSim (default: 8000)",
    )
    argparser.add_argument(
        "--terasim_config",
        default="examples/simulation_Mcity_carla_config.yaml",
        help="Configuration file path for TeraSim",
    )
    argparser.add_argument(
        "--local_align_tolerance",
        default=0.10,
        type=float,
        help="Allowed relative span mismatch for local bounds alignment (default: 0.10)",
    )
    return argparser.parse_args()


def _relative_span_error(a_min, a_max, b_min, b_max):
    a_span = abs(a_max - a_min)
    b_span = abs(b_max - b_min)
    denom = max(a_span, b_span, 1e-6)
    return abs(a_span - b_span) / denom


def _sumo_bounds_from_net(net_file):
    root = ET.parse(net_file).getroot()
    loc_elem = root.find(".//location")
    proj_parameter = None
    if loc_elem is not None:
        proj_parameter = loc_elem.get("projParameter")
        conv_boundary = loc_elem.get("convBoundary")
        if conv_boundary:
            x_min, y_min, x_max, y_max = map(float, conv_boundary.split(","))
            return proj_parameter, (x_min, y_min, x_max, y_max)

    xs = []
    ys = []
    for lane in root.iter("lane"):
        shape = lane.get("shape")
        if not shape:
            continue
        for point in shape.split():
            values = point.split(",")
            xs.append(float(values[0]))
            ys.append(float(values[1]))

    if not xs or not ys:
        raise RuntimeError(f"Could not derive SUMO bounds from {net_file}")
    return proj_parameter, (min(xs), min(ys), max(xs), max(ys))


def _carla_waypoint_bounds(world):
    xs = []
    ys = []
    for waypoint in world.get_map().generate_waypoints(50.0):
        loc = waypoint.transform.location
        xs.append(loc.x)
        ys.append(loc.y)

    if not xs or not ys:
        raise RuntimeError("Could not derive CARLA waypoint bounds")
    return min(xs), min(ys), max(xs), max(ys)


def maybe_apply_local_bounds_alignment(carla_cosim: CarlaCosim, tolerance: float) -> None:
    explicit_offset_x = os.environ.get("SUMO_TO_CARLA_OFFSET_X")
    explicit_offset_y = os.environ.get("SUMO_TO_CARLA_OFFSET_Y")
    explicit_offset_z = os.environ.get("SUMO_TO_CARLA_OFFSET_Z")
    if explicit_offset_x is not None and explicit_offset_y is not None:
        offset_x = float(explicit_offset_x)
        offset_y = float(explicit_offset_y)
        offset_z = float(explicit_offset_z or "0.0")
        carla_cosim._coord_transformer = None
        carla_cosim.sumo_carla_offset = [offset_x, offset_y]
        print("Using explicit SUMO -> CARLA local offset from environment")
        print(
            f"  overriding offset: dx={offset_x:.2f}, "
            f"dy={offset_y:.2f}, dz={offset_z:.2f}"
        )
        return

    net_file = carla_cosim._get_net_file_from_config(carla_cosim.args.terasim_config)
    if not net_file:
        print("Local alignment skipped: no SUMO net file found in config")
        return

    try:
        proj_parameter, (sx_min, sy_min, sx_max, sy_max) = _sumo_bounds_from_net(net_file)
    except Exception as exc:
        print(f"Local alignment skipped: failed to parse SUMO bounds ({exc})")
        return

    if proj_parameter != "!":
        print(f"Local alignment skipped: SUMO projParameter is {proj_parameter!r}, not '!'.")
        return

    try:
        cx_min, cy_min, cx_max, cy_max = _carla_waypoint_bounds(carla_cosim.world)
    except Exception as exc:
        print(f"Local alignment skipped: failed to derive CARLA waypoint bounds ({exc})")
        return

    x_span_error = _relative_span_error(sx_min, sx_max, cx_min, cx_max)
    y_span_error = _relative_span_error(sy_min, sy_max, cy_min, cy_max)
    if x_span_error > tolerance or y_span_error > tolerance:
        print(
            "Local alignment skipped: SUMO/CARLA spans differ too much "
            f"(x_error={x_span_error:.3f}, y_error={y_span_error:.3f})"
        )
        return

    sumo_center_x = (sx_min + sx_max) / 2.0
    sumo_center_y = (sy_min + sy_max) / 2.0
    carla_center_x = (cx_min + cx_max) / 2.0
    carla_center_y = (cy_min + cy_max) / 2.0

    offset_x = carla_center_x - sumo_center_x
    offset_y = carla_center_y + sumo_center_y

    carla_cosim._coord_transformer = None
    carla_cosim.sumo_carla_offset = [offset_x, offset_y]

    print("Using local bounds alignment for SUMO -> CARLA")
    print(
        "  SUMO bounds: "
        f"x=[{sx_min:.2f}, {sx_max:.2f}] y=[{sy_min:.2f}, {sy_max:.2f}]"
    )
    print(
        "  CARLA bounds: "
        f"x=[{cx_min:.2f}, {cx_max:.2f}] y=[{cy_min:.2f}, {cy_max:.2f}]"
    )
    print(f"  span errors: x={x_span_error:.4f}, y={y_span_error:.4f}")
    print(f"  overriding offset: dx={offset_x:.2f}, dy={offset_y:.2f}")


class MotionDiagnostics:
    fieldnames = [
        "wall_time",
        "sim_time",
        "frame",
        "veh_id",
        "carla_id",
        "sumo_x",
        "sumo_y",
        "sumo_z",
        "sumo_angle",
        "sumo_speed",
        "target_x",
        "target_y",
        "target_z",
        "target_yaw",
        "actual_x",
        "actual_y",
        "actual_z",
        "actual_yaw",
        "target_error",
        "dx",
        "dy",
        "dz",
        "dt",
        "step_distance",
        "signed_forward_delta",
        "estimated_forward_speed",
        "backward",
    ]

    def __init__(
        self,
        carla_cosim: CarlaCosim,
        log_path: str,
        role_names: set[str] | None,
        backward_threshold: float,
        min_movement: float,
    ) -> None:
        self.carla_cosim = carla_cosim
        self.log_path = log_path
        self.role_names = role_names
        self.backward_threshold = backward_threshold
        self.min_movement = min_movement
        self.last_by_vehicle: dict[str, dict[str, float]] = {}
        self.stats: dict[str, dict[str, float]] = {}

        directory = os.path.dirname(log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.file = open(log_path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.file.flush()

    @staticmethod
    def _parse_role_names(value: str) -> set[str] | None:
        value = value.strip()
        if value == "" or value.lower() in {"*", "all"}:
            return None
        return {item.strip() for item in value.split(",") if item.strip()}

    @classmethod
    def from_environment(cls, carla_cosim: CarlaCosim) -> "MotionDiagnostics | None":
        log_path = os.environ.get("CARLA_COSIM_MOTION_LOG", "").strip()
        if not log_path:
            return None

        role_names = cls._parse_role_names(
            os.environ.get("CARLA_COSIM_DIAG_ROLE_NAMES", "AV")
        )
        backward_threshold = float(
            os.environ.get("CARLA_COSIM_DIAG_BACKWARD_THRESHOLD", "-0.05")
        )
        min_movement = float(os.environ.get("CARLA_COSIM_DIAG_MIN_MOVEMENT", "0.01"))
        diagnostics = cls(carla_cosim, log_path, role_names, backward_threshold, min_movement)

        roles_text = "all vehicles" if role_names is None else ", ".join(sorted(role_names))
        print("CARLA co-sim motion diagnostics enabled")
        print(f"  log: {log_path}")
        print(f"  vehicles: {roles_text}")
        print(f"  backward threshold: {backward_threshold:.3f} m")
        return diagnostics

    def install(self) -> None:
        original_process_vehicle = self.carla_cosim._process_vehicle

        def wrapped_process_vehicle(veh_id, veh_info, cosim_id_record, *args, **kwargs):
            original_process_vehicle(veh_id, veh_info, cosim_id_record, *args, **kwargs)
            self.record_vehicle(veh_id, veh_info)

        self.carla_cosim._process_vehicle = wrapped_process_vehicle

    def _should_record(self, veh_id: str) -> bool:
        return self.role_names is None or veh_id in self.role_names

    def _stats_for(self, veh_id: str) -> dict[str, float]:
        return self.stats.setdefault(
            veh_id,
            {
                "samples": 0,
                "moving_samples": 0,
                "total_distance": 0.0,
                "total_signed_forward_delta": 0.0,
                "backward_events": 0,
                "max_step_distance": 0.0,
                "min_signed_forward_delta": math.inf,
            },
        )

    def record_vehicle(self, veh_id: str, veh_info: dict) -> None:
        if not self._should_record(veh_id):
            return

        vehicle_status, carla_id = get_actor_id_from_attribute(self.carla_cosim.world, veh_id)
        if not vehicle_status:
            return

        vehicle = self.carla_cosim.world.get_actor(carla_id)
        if vehicle is None:
            return

        sumo_location = [veh_info["x"], veh_info["y"], veh_info["z"]]
        sumo_rotation = [0.0, veh_info["sumo_angle"], 0.0]
        shape = [veh_info["length"], veh_info["width"], veh_info["height"]]
        sumo_offset = self.carla_cosim._get_carla_offset(sumo_location, 0.0)
        target_transform = sumo_to_carla(sumo_location, sumo_rotation, shape, sumo_offset)
        actual_transform = vehicle.get_transform()
        target_location = target_transform.location
        actual_location = actual_transform.location

        snapshot = self.carla_cosim.world.get_snapshot()
        wall_time = time.time()
        sim_time = snapshot.timestamp.elapsed_seconds
        frame = snapshot.frame
        previous = self.last_by_vehicle.get(veh_id)

        dx = dy = dz = dt = step_distance = signed_forward_delta = estimated_speed = ""
        backward = 0
        if previous is not None:
            dx_value = actual_location.x - previous["x"]
            dy_value = actual_location.y - previous["y"]
            dz_value = actual_location.z - previous["z"]
            dt_value = sim_time - previous["sim_time"]
            if dt_value <= 0:
                dt_value = wall_time - previous["wall_time"]

            distance_value = math.sqrt(
                dx_value * dx_value + dy_value * dy_value + dz_value * dz_value
            )
            previous_yaw_rad = math.radians(previous["yaw"])
            signed_value = dx_value * math.cos(previous_yaw_rad) + dy_value * math.sin(
                previous_yaw_rad
            )
            estimated_value = signed_value / dt_value if dt_value > 0 else 0.0
            if distance_value > self.min_movement and signed_value < self.backward_threshold:
                backward = 1

            dx = dx_value
            dy = dy_value
            dz = dz_value
            dt = dt_value
            step_distance = distance_value
            signed_forward_delta = signed_value
            estimated_speed = estimated_value

            stats = self._stats_for(veh_id)
            stats["moving_samples"] += int(distance_value > self.min_movement)
            stats["total_distance"] += distance_value
            stats["total_signed_forward_delta"] += signed_value
            stats["backward_events"] += backward
            stats["max_step_distance"] = max(stats["max_step_distance"], distance_value)
            stats["min_signed_forward_delta"] = min(
                stats["min_signed_forward_delta"], signed_value
            )

        target_error = math.sqrt(
            (actual_location.x - target_location.x) ** 2
            + (actual_location.y - target_location.y) ** 2
            + (actual_location.z - target_location.z) ** 2
        )

        self._stats_for(veh_id)["samples"] += 1
        self.writer.writerow(
            {
                "wall_time": wall_time,
                "sim_time": sim_time,
                "frame": frame,
                "veh_id": veh_id,
                "carla_id": carla_id,
                "sumo_x": veh_info["x"],
                "sumo_y": veh_info["y"],
                "sumo_z": veh_info["z"],
                "sumo_angle": veh_info["sumo_angle"],
                "sumo_speed": veh_info.get("speed", ""),
                "target_x": target_location.x,
                "target_y": target_location.y,
                "target_z": target_location.z,
                "target_yaw": target_transform.rotation.yaw,
                "actual_x": actual_location.x,
                "actual_y": actual_location.y,
                "actual_z": actual_location.z,
                "actual_yaw": actual_transform.rotation.yaw,
                "target_error": target_error,
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "dt": dt,
                "step_distance": step_distance,
                "signed_forward_delta": signed_forward_delta,
                "estimated_forward_speed": estimated_speed,
                "backward": backward,
            }
        )
        self.file.flush()

        self.last_by_vehicle[veh_id] = {
            "wall_time": wall_time,
            "sim_time": sim_time,
            "x": actual_location.x,
            "y": actual_location.y,
            "z": actual_location.z,
            "yaw": actual_transform.rotation.yaw,
        }

    def close(self) -> None:
        if self.file.closed:
            return

        print("CARLA co-sim motion diagnostics summary")
        total_backward = 0
        for veh_id, stats in sorted(self.stats.items()):
            total_backward += int(stats["backward_events"])
            min_signed = stats["min_signed_forward_delta"]
            min_signed_text = "n/a" if min_signed == math.inf else f"{min_signed:.4f}m"
            print(
                f"  {veh_id}: samples={int(stats['samples'])} "
                f"moving={int(stats['moving_samples'])} "
                f"distance={stats['total_distance']:.2f}m "
                f"signed={stats['total_signed_forward_delta']:.2f}m "
                f"backward_events={int(stats['backward_events'])} "
                f"min_signed={min_signed_text} "
                f"max_step={stats['max_step_distance']:.2f}m"
            )
        if total_backward == 0:
            print("  verdict: no backward set_transform steps were recorded.")
        else:
            print(
                "  verdict: backward set_transform steps were recorded; "
                "inspect backward=1 rows."
            )
        print(f"  csv: {self.log_path}")
        self.file.close()


class ActorSyncProfiler:
    fieldnames = [
        "wall_time",
        "frame",
        "sim_time",
        "state_get",
        "validate",
        "actor_index_build",
        "vehicle_count",
        "vru_count",
        "vehicle_loop",
        "vehicle_process_calls",
        "vehicle_process_max",
        "vru_loop",
        "vru_process_calls",
        "vru_process_max",
        "cleanup_vehicle",
        "cleanup_pedestrian",
        "lookup_calls",
        "lookup_total",
        "lookup_max",
        "sumo_to_carla_calls",
        "sumo_to_carla_total",
        "sumo_to_carla_max",
        "spawn_calls",
        "spawn_total",
        "spawn_max",
        "total",
        "result",
    ]

    def __init__(self, carla_cosim: CarlaCosim, log_path: str, print_every: int) -> None:
        self.carla_cosim = carla_cosim
        self.log_path = log_path
        self.print_every = max(print_every, 0)
        self.rows_written = 0
        self.active_row: dict | None = None
        self.stage_values: dict[str, list[float]] = {
            key: []
            for key in [
                "state_get",
                "actor_index_build",
                "vehicle_loop",
                "vru_loop",
                "cleanup_vehicle",
                "cleanup_pedestrian",
                "lookup_total",
                "sumo_to_carla_total",
                "spawn_total",
                "total",
            ]
        }

        self.original_sync_actor = carla_cosim.sync_cosim_actor_to_carla
        self.original_lookup = carla_cosim_module.get_actor_id_from_attribute
        self.original_sumo_to_carla = carla_cosim_module.sumo_to_carla
        self.original_spawn_actor = carla_cosim_module.spawn_actor

        directory = os.path.dirname(log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.file = open(log_path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.file.flush()

    @classmethod
    def from_environment(cls, carla_cosim: CarlaCosim) -> "ActorSyncProfiler | None":
        log_path = os.environ.get("CARLA_COSIM_ACTOR_PROFILE_LOG", "").strip()
        if not log_path:
            return None
        print_every = int(os.environ.get("CARLA_COSIM_ACTOR_PROFILE_PRINT_EVERY", "20"))
        profiler = cls(carla_cosim, log_path, print_every)
        print("CARLA co-sim actor sync profiler enabled")
        print(f"  log: {log_path}")
        print(f"  print every: {print_every} actor syncs")
        return profiler

    @staticmethod
    def _elapsed(start: float) -> float:
        return time.perf_counter() - start

    def install(self) -> None:
        def timed_lookup(world, attribute):
            start = time.perf_counter()
            try:
                return self.original_lookup(world, attribute)
            finally:
                elapsed = self._elapsed(start)
                row = self.active_row
                if row is not None:
                    row["lookup_calls"] += 1
                    row["lookup_total"] += elapsed
                    row["lookup_max"] = max(row["lookup_max"], elapsed)

        def timed_sumo_to_carla(sumo_location, sumo_rotation, shape, offset):
            start = time.perf_counter()
            try:
                return self.original_sumo_to_carla(sumo_location, sumo_rotation, shape, offset)
            finally:
                elapsed = self._elapsed(start)
                row = self.active_row
                if row is not None:
                    row["sumo_to_carla_calls"] += 1
                    row["sumo_to_carla_total"] += elapsed
                    row["sumo_to_carla_max"] = max(row["sumo_to_carla_max"], elapsed)

        def timed_spawn_actor(*args, **kwargs):
            start = time.perf_counter()
            try:
                return self.original_spawn_actor(*args, **kwargs)
            finally:
                elapsed = self._elapsed(start)
                row = self.active_row
                if row is not None:
                    row["spawn_calls"] += 1
                    row["spawn_total"] += elapsed
                    row["spawn_max"] = max(row["spawn_max"], elapsed)

        carla_cosim_module.get_actor_id_from_attribute = timed_lookup
        carla_cosim_module.sumo_to_carla = timed_sumo_to_carla
        carla_cosim_module.spawn_actor = timed_spawn_actor
        self.carla_cosim.sync_cosim_actor_to_carla = self.profiled_sync_actor

    def _new_row(self) -> dict:
        snapshot = self.carla_cosim.world.get_snapshot()
        return {
            "wall_time": time.time(),
            "frame": snapshot.frame,
            "sim_time": snapshot.timestamp.elapsed_seconds,
            "state_get": 0.0,
            "validate": 0.0,
            "actor_index_build": 0.0,
            "vehicle_count": 0,
            "vru_count": 0,
            "vehicle_loop": 0.0,
            "vehicle_process_calls": 0,
            "vehicle_process_max": 0.0,
            "vru_loop": 0.0,
            "vru_process_calls": 0,
            "vru_process_max": 0.0,
            "cleanup_vehicle": 0.0,
            "cleanup_pedestrian": 0.0,
            "lookup_calls": 0,
            "lookup_total": 0.0,
            "lookup_max": 0.0,
            "sumo_to_carla_calls": 0,
            "sumo_to_carla_total": 0.0,
            "sumo_to_carla_max": 0.0,
            "spawn_calls": 0,
            "spawn_total": 0.0,
            "spawn_max": 0.0,
            "total": 0.0,
            "result": "ok",
        }

    def _record(self, row: dict) -> None:
        self.rows_written += 1
        for key, values in self.stage_values.items():
            value = row.get(key)
            if isinstance(value, (int, float)):
                values.append(float(value))
        self.writer.writerow(row)
        self.file.flush()

        if self.print_every and self.rows_written % self.print_every == 0:
            print(
                "actor sync profile: "
                f"rows={self.rows_written} "
                f"total={row['total']:.3f}s "
                f"state_get={row['state_get']:.3f}s "
                f"index={row['actor_index_build']:.3f}s "
                f"vehicle_loop={row['vehicle_loop']:.3f}s "
                f"cleanup={row['cleanup_vehicle'] + row['cleanup_pedestrian']:.3f}s "
                f"lookup={row['lookup_total']:.3f}s/{row['lookup_calls']}calls"
            )

    def profiled_sync_actor(self):
        cosim = self.carla_cosim
        row = self._new_row()
        total_start = time.perf_counter()
        self.active_row = row
        try:
            stage_start = time.perf_counter()
            terasim_states = carla_cosim_module.get_terasim_states(
                cosim.args.terasim_host,
                cosim.args.terasim_port,
                cosim.terasim["simulation_id"],
            )
            row["state_get"] = self._elapsed(stage_start)

            validate_start = time.perf_counter()
            if not terasim_states:
                print("terasim_states not available.")
                row["result"] = "no_states"
                return
            if "agent_details" not in terasim_states:
                print("No agent details available.")
                row["result"] = "no_agent_details"
                return
            if "vehicle" not in terasim_states["agent_details"]:
                print("No vehicle details available.")
                row["result"] = "no_vehicle_details"
                return
            if "vru" not in terasim_states["agent_details"]:
                print("No VRU details available.")
                row["result"] = "no_vru_details"
                return
            row["validate"] = self._elapsed(validate_start)

            vehicles = terasim_states["agent_details"]["vehicle"]
            vrus = terasim_states["agent_details"]["vru"]
            row["vehicle_count"] = len(vehicles)
            row["vru_count"] = len(vrus)

            cosim_id_record = set()
            current_frame = cosim.world.get_snapshot().frame

            stage_start = time.perf_counter()
            vehicle_actor_index, pedestrian_actor_index = cosim._build_actor_role_indexes()
            row["actor_index_build"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            for veh_id, veh_info in vehicles.items():
                if cosim.control_av and veh_id == carla_cosim_module.AV_SUMO_ID:
                    if cosim.initialize_av:
                        continue
                    cosim.initialize_av = True
                    cosim.av_shape = [
                        veh_info["length"],
                        veh_info["width"],
                        veh_info["height"],
                    ]
                    print("AV is initialized based on SUMO state.")
                    print(veh_info)

                process_start = time.perf_counter()
                cosim._process_vehicle(
                    veh_id,
                    veh_info,
                    cosim_id_record,
                    carla_actor=vehicle_actor_index.get(veh_id),
                    actor_index=vehicle_actor_index,
                    current_frame=current_frame,
                )
                elapsed = self._elapsed(process_start)
                row["vehicle_process_calls"] += 1
                row["vehicle_process_max"] = max(row["vehicle_process_max"], elapsed)
            row["vehicle_loop"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            for vru_id, vru_info in vrus.items():
                vru_actor_index = (
                    vehicle_actor_index
                    if cosim._vru_uses_vehicle_blueprint(vru_info)
                    else pedestrian_actor_index
                )
                process_start = time.perf_counter()
                cosim._process_vru(
                    vru_id,
                    vru_info,
                    cosim_id_record,
                    carla_actor=vru_actor_index.get(vru_id),
                    actor_index=vru_actor_index,
                    current_frame=current_frame,
                )
                elapsed = self._elapsed(process_start)
                row["vru_process_calls"] += 1
                row["vru_process_max"] = max(row["vru_process_max"], elapsed)
            row["vru_loop"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
            row["cleanup_vehicle"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)
            row["cleanup_pedestrian"] = self._elapsed(stage_start)

            cosim._prune_spawn_failures(vehicles.keys(), vrus.keys())
        finally:
            row["total"] = self._elapsed(total_start)
            self.active_row = None
            self._record(row)

    def close(self) -> None:
        carla_cosim_module.get_actor_id_from_attribute = self.original_lookup
        carla_cosim_module.sumo_to_carla = self.original_sumo_to_carla
        carla_cosim_module.spawn_actor = self.original_spawn_actor
        self.carla_cosim.sync_cosim_actor_to_carla = self.original_sync_actor

        if self.file.closed:
            return

        print("CARLA co-sim actor sync profiler summary")
        for key, values in self.stage_values.items():
            if not values:
                continue
            print(
                f"  {key}: "
                f"median={statistics.median(values):.3f}s "
                f"mean={statistics.mean(values):.3f}s "
                f"max={max(values):.3f}s"
            )
        print(f"  csv: {self.log_path}")
        self.file.close()


class TickProfiler:
    fieldnames = [
        "wall_time",
        "frame_before",
        "frame_after",
        "sim_time_before",
        "sim_time_after",
        "status_wait",
        "status_polls",
        "sync_av",
        "sync_actor",
        "sync_tls",
        "tick_terasim",
        "world_tick",
        "async_sleep",
        "total",
        "result",
    ]

    def __init__(self, carla_cosim: CarlaCosim, log_path: str, print_every: int) -> None:
        self.carla_cosim = carla_cosim
        self.log_path = log_path
        self.print_every = max(print_every, 0)
        self.rows_written = 0
        self.stage_values: dict[str, list[float]] = {
            key: []
            for key in [
                "status_wait",
                "sync_av",
                "sync_actor",
                "sync_tls",
                "tick_terasim",
                "world_tick",
                "async_sleep",
                "total",
            ]
        }

        directory = os.path.dirname(log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.file = open(log_path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.file.flush()

    @classmethod
    def from_environment(cls, carla_cosim: CarlaCosim) -> "TickProfiler | None":
        log_path = os.environ.get("CARLA_COSIM_PROFILE_LOG", "").strip()
        if not log_path:
            return None
        print_every = int(os.environ.get("CARLA_COSIM_PROFILE_PRINT_EVERY", "20"))
        profiler = cls(carla_cosim, log_path, print_every)
        print("CARLA co-sim tick profiler enabled")
        print(f"  log: {log_path}")
        print(f"  print every: {print_every} ticks")
        return profiler

    @staticmethod
    def _elapsed(start: float) -> float:
        return time.perf_counter() - start

    def install(self) -> None:
        self.carla_cosim.tick = self.tick

    def _snapshot_values(self) -> tuple[int, float]:
        snapshot = self.carla_cosim.world.get_snapshot()
        return snapshot.frame, snapshot.timestamp.elapsed_seconds

    def _record(self, row: dict) -> None:
        self.rows_written += 1
        for key, values in self.stage_values.items():
            value = row.get(key)
            if isinstance(value, (int, float)):
                values.append(float(value))
        self.writer.writerow(row)
        self.file.flush()

        if self.print_every and self.rows_written % self.print_every == 0:
            total_values = self.stage_values["total"]
            latest_total = total_values[-1] if total_values else 0.0
            print(
                "tick profile: "
                f"ticks={self.rows_written} "
                f"latest_total={latest_total:.3f}s "
                f"status_wait={row.get('status_wait', 0.0):.3f}s "
                f"sync_actor={row.get('sync_actor', 0.0):.3f}s "
                f"tick_terasim={row.get('tick_terasim', 0.0):.3f}s "
                f"world_tick={row.get('world_tick', 0.0):.3f}s"
            )

    def tick(self) -> bool:
        cosim = self.carla_cosim
        total_start = time.perf_counter()
        frame_before, sim_time_before = self._snapshot_values()
        row = {
            "wall_time": time.time(),
            "frame_before": frame_before,
            "sim_time_before": sim_time_before,
            "status_wait": 0.0,
            "status_polls": 0,
            "sync_av": 0.0,
            "sync_actor": 0.0,
            "sync_tls": 0.0,
            "tick_terasim": 0.0,
            "world_tick": 0.0,
            "async_sleep": 0.0,
            "result": "ok",
        }

        if cosim.async_mode:
            loop_start = time.perf_counter()
            if cosim.control_av:
                stage_start = time.perf_counter()
                cosim.sync_carla_av_to_cosim()
                row["sync_av"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.sync_cosim_actor_to_carla()
            row["sync_actor"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.sync_cosim_tls_to_carla()
            row["sync_tls"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.world.tick()
            row["world_tick"] = self._elapsed(stage_start)

            elapsed = time.perf_counter() - loop_start
            if elapsed < cosim.step_length:
                sleep_time = cosim.step_length - elapsed
                time.sleep(sleep_time)
                row["async_sleep"] = sleep_time
        else:
            stage_start = time.perf_counter()
            while True:
                status_response = carla_cosim_module.get_terasim_status(
                    cosim.args.terasim_host,
                    cosim.args.terasim_port,
                    cosim.terasim["simulation_id"],
                )
                row["status_polls"] += 1
                terasim_status = status_response.get("status", None)
                if terasim_status in {"ticked", "wait_for_tick"}:
                    break
                if terasim_status is None:
                    print("TeraSim status is None. Exiting...")
                    row["status_wait"] = self._elapsed(stage_start)
                    row["result"] = "terasim_status_none"
                    row["total"] = self._elapsed(total_start)
                    frame_after, sim_time_after = self._snapshot_values()
                    row["frame_after"] = frame_after
                    row["sim_time_after"] = sim_time_after
                    self._record(row)
                    return False
                time.sleep(0.05)
            row["status_wait"] = self._elapsed(stage_start)

            if cosim.control_av:
                stage_start = time.perf_counter()
                cosim.sync_carla_av_to_cosim()
                row["sync_av"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.sync_cosim_actor_to_carla()
            row["sync_actor"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.sync_cosim_tls_to_carla()
            row["sync_tls"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            carla_cosim_module.tick_terasim(
                cosim.args.terasim_host,
                cosim.args.terasim_port,
                cosim.terasim["simulation_id"],
            )
            row["tick_terasim"] = self._elapsed(stage_start)

            stage_start = time.perf_counter()
            cosim.world.tick()
            row["world_tick"] = self._elapsed(stage_start)

        frame_after, sim_time_after = self._snapshot_values()
        row["frame_after"] = frame_after
        row["sim_time_after"] = sim_time_after
        row["total"] = self._elapsed(total_start)
        self._record(row)
        return True

    def close(self) -> None:
        if self.file.closed:
            return

        print("CARLA co-sim tick profiler summary")
        for key, values in self.stage_values.items():
            if not values:
                continue
            print(
                f"  {key}: "
                f"median={statistics.median(values):.3f}s "
                f"mean={statistics.mean(values):.3f}s "
                f"max={max(values):.3f}s"
            )
        print(f"  csv: {self.log_path}")
        self.file.close()


def main():
    args = parse_args()
    carla_cosim = CarlaCosim(args)
    maybe_apply_local_bounds_alignment(carla_cosim, args.local_align_tolerance)
    motion_diagnostics = MotionDiagnostics.from_environment(carla_cosim)
    if motion_diagnostics is not None:
        motion_diagnostics.install()
    actor_sync_profiler = ActorSyncProfiler.from_environment(carla_cosim)
    if actor_sync_profiler is not None:
        actor_sync_profiler.install()
    tick_profiler = TickProfiler.from_environment(carla_cosim)
    if tick_profiler is not None:
        tick_profiler.install()

    if not args.async_mode:
        for attempt in range(5):
            settings = carla_cosim.world.get_settings()
            settings.fixed_delta_seconds = args.step_length
            settings.synchronous_mode = True
            carla_cosim.world.apply_settings(settings)
            time.sleep(1.0)
            current = carla_cosim.world.get_settings()
            if current.synchronous_mode:
                print(f"Synchronous mode enabled (attempt {attempt + 1})")
                break
            print(f"Settings not applied yet, retrying... (attempt {attempt + 1})")
            try:
                carla_cosim.world.tick()
            except Exception:
                pass
            time.sleep(2.0)
    else:
        settings = carla_cosim.world.get_settings()
        if settings.synchronous_mode:
            try:
                carla_cosim.world.tick()
            except Exception:
                pass
            time.sleep(0.5)
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            carla_cosim.world.apply_settings(settings)
            time.sleep(1.0)
        print("Running in async mode")

    carla_cosim.world.set_weather(carla.WeatherParameters.WetSunset)

    try:
        tick_flag = True
        while tick_flag:
            tick_flag = carla_cosim.tick()
    except KeyboardInterrupt:
        print("Cancelled by user.")
    finally:
        if tick_profiler is not None:
            tick_profiler.close()
        if actor_sync_profiler is not None:
            actor_sync_profiler.close()
        if motion_diagnostics is not None:
            motion_diagnostics.close()
        print("Cleaning synchronization")
        carla_cosim.close()


if __name__ == "__main__":
    main()

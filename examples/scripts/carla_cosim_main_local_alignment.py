"""CARLA Co-Simulation Client with local-bounds alignment fallback.

This variant keeps the standard CarlaCosim flow, but overrides the
SUMO->CARLA XY offset when the SUMO net.xml has no projection metadata
(`projParameter='!'`) and its metric bounds already match the loaded
CARLA map shape. This is useful for custom packaged maps where both
SUMO and CARLA share the same local metric frame but differ by a fixed
translation and axis flip.
"""
import argparse
import os
import time
import xml.etree.ElementTree as ET

import carla
from terasim_service.utils.carla import CarlaCosim


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
        print(f"  overriding offset: dx={offset_x:.2f}, dy={offset_y:.2f}, dz={offset_z:.2f}")
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


def main():
    args = parse_args()
    carla_cosim = CarlaCosim(args)
    maybe_apply_local_bounds_alignment(carla_cosim, args.local_align_tolerance)

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
        print("Cleaning synchronization")
        carla_cosim.close()


if __name__ == "__main__":
    main()

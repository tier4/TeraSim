"""Apply simple CARLA-side controls to the co-sim AV actor.

This helper is intended to run alongside the TeraSim/CARLA co-simulation
client. It never ticks the CARLA world; the co-sim client remains the only
world tick owner in synchronous mode.
"""

import argparse
import csv
import math
import time
from pathlib import Path

import carla


def parse_args():
    parser = argparse.ArgumentParser(description="Control the CARLA AV actor during co-sim")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", default=2000, type=int, help="CARLA port")
    parser.add_argument("--timeout", default=30.0, type=float, help="CARLA RPC timeout [s]")
    parser.add_argument("--role-name", default="AV", help="CARLA actor role_name to control")
    parser.add_argument("--duration", default=30.0, type=float, help="Control duration [s]")
    parser.add_argument("--throttle", default=0.25, type=float, help="Throttle command [0, 1]")
    parser.add_argument("--steer", default=0.0, type=float, help="Steer command [-1, 1]")
    parser.add_argument("--brake", default=0.0, type=float, help="Brake command [0, 1]")
    parser.add_argument(
        "--settle-brake-seconds",
        default=1.0,
        type=float,
        help="Brake duration after the control phase [s]",
    )
    parser.add_argument(
        "--no-enable-physics",
        action="store_true",
        help="Do not call set_simulate_physics(True) before applying controls",
    )
    parser.add_argument("--log-csv", default="", help="Optional CSV log path")
    parser.add_argument(
        "--print-every",
        default=30,
        type=int,
        help="Print every N observed CARLA frames",
    )
    return parser.parse_args()


def find_actor_by_role(world, role_name):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def wait_for_actor(world, role_name, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        actor = find_actor_by_role(world, role_name)
        if actor is not None:
            return actor
        try:
            world.wait_for_tick(seconds=1.0)
        except RuntimeError:
            time.sleep(0.2)
    raise RuntimeError(f"CARLA actor with role_name={role_name!r} was not found")


def speed_mps(actor):
    velocity = actor.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def main():
    args = parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()

    settings = world.get_settings()
    print(
        "Connected to CARLA. "
        f"sync={settings.synchronous_mode} "
        f"fixed_delta_seconds={settings.fixed_delta_seconds}",
        flush=True,
    )

    actor = wait_for_actor(world, args.role_name, args.timeout)
    print(f"Found actor id={actor.id} role_name={args.role_name}", flush=True)

    if not args.no_enable_physics:
        actor.set_simulate_physics(True)
        print("Enabled CARLA physics for AV actor", flush=True)

    writer = None
    log_file = None
    if args.log_csv:
        log_path = Path(args.log_csv)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = log_path.open("w", newline="")
        writer = csv.DictWriter(
            log_file,
            fieldnames=[
                "wall_time",
                "frame",
                "elapsed",
                "x",
                "y",
                "z",
                "yaw",
                "speed_mps",
                "throttle",
                "steer",
                "brake",
            ],
        )
        writer.writeheader()

    start = time.monotonic()
    frames = 0
    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= args.duration:
                break

            control = carla.VehicleControl(
                throttle=max(0.0, min(1.0, args.throttle)),
                steer=max(-1.0, min(1.0, args.steer)),
                brake=max(0.0, min(1.0, args.brake)),
            )
            actor.apply_control(control)
            snapshot = world.wait_for_tick(seconds=max(args.timeout, 1.0))
            frames += 1

            transform = actor.get_transform()
            speed = speed_mps(actor)
            if writer is not None:
                writer.writerow(
                    {
                        "wall_time": time.time(),
                        "frame": snapshot.frame,
                        "elapsed": elapsed,
                        "x": transform.location.x,
                        "y": transform.location.y,
                        "z": transform.location.z,
                        "yaw": transform.rotation.yaw,
                        "speed_mps": speed,
                        "throttle": control.throttle,
                        "steer": control.steer,
                        "brake": control.brake,
                    }
                )

            if args.print_every > 0 and frames % args.print_every == 0:
                print(
                    f"frame={snapshot.frame} elapsed={elapsed:.2f}s "
                    f"speed={speed:.2f}m/s "
                    f"loc=({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})",
                    flush=True,
                )

        if args.settle_brake_seconds > 0.0:
            stop_until = time.monotonic() + args.settle_brake_seconds
            while time.monotonic() < stop_until:
                actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                world.wait_for_tick(seconds=max(args.timeout, 1.0))
    finally:
        actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        if log_file is not None:
            log_file.close()
            print(f"Wrote control log: {args.log_csv}", flush=True)

    print("AV control script finished", flush=True)


if __name__ == "__main__":
    main()

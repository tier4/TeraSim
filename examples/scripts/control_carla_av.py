"""Apply CARLA-side controls to the co-sim AV actor.

This helper is intended to run alongside the TeraSim/CARLA co-simulation
client. It never ticks the CARLA world; the co-sim client remains the only
world tick owner in synchronous mode.
"""

import argparse
import contextlib
import csv
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

carla = None


def parse_args():
    parser = argparse.ArgumentParser(description="Control the CARLA AV actor during co-sim")
    parser.add_argument("--host", default="127.0.0.1", help="CARLA host")
    parser.add_argument("--port", default=2000, type=int, help="CARLA port")
    parser.add_argument("--timeout", default=30.0, type=float, help="CARLA RPC timeout [s]")
    parser.add_argument("--role-name", default="AV", help="CARLA actor role_name to control")
    parser.add_argument(
        "--duration",
        default=30.0,
        type=float,
        help="Control duration [s]. Use 0 or a negative value to run until quit.",
    )
    parser.add_argument("--throttle", default=0.25, type=float, help="Throttle command [0, 1]")
    parser.add_argument("--steer", default=0.0, type=float, help="Steer command [-1, 1]")
    parser.add_argument("--brake", default=0.0, type=float, help="Brake command [0, 1]")
    parser.add_argument(
        "--keyboard",
        action="store_true",
        help="Read terminal keyboard input and update AV controls interactively",
    )
    parser.add_argument(
        "--keyboard-throttle-step",
        default=0.05,
        type=float,
        help="Throttle increment for each w key press",
    )
    parser.add_argument(
        "--keyboard-brake-step",
        default=0.10,
        type=float,
        help="Brake increment for each s key press",
    )
    parser.add_argument(
        "--keyboard-steer-step",
        default=0.05,
        type=float,
        help="Steer increment for each a/d key press",
    )
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


@contextlib.contextmanager
def raw_terminal(enabled):
    if not enabled:
        yield
        return
    if not sys.stdin.isatty():
        raise RuntimeError(
            "--keyboard requires an interactive terminal. Use docker compose run -it."
        )
    original = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)


def read_available_keys():
    keys = []
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            break
        keys.append(sys.stdin.read(1))
    return keys


def print_keyboard_help():
    print(
        "\nKeyboard control:\n"
        "  w: throttle up      s: brake up\n"
        "  a: steer left       d: steer right\n"
        "  c: coast/reset throttle, brake, and steer\n"
        "  space/x: full brake\n"
        "  r: toggle reverse\n"
        "  h: show this help\n"
        "  q or Esc: quit\n",
        flush=True,
    )


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_keyboard_input(args, state):
    quit_requested = False
    for key in read_available_keys():
        key = key.lower()
        if key in {"q", "\x1b"}:
            quit_requested = True
        elif key == "h":
            print_keyboard_help()
        elif key == "w":
            state["throttle"] = clamp(
                state["throttle"] + args.keyboard_throttle_step, 0.0, 1.0
            )
            state["brake"] = 0.0
        elif key == "s":
            state["brake"] = clamp(state["brake"] + args.keyboard_brake_step, 0.0, 1.0)
            state["throttle"] = 0.0
        elif key == "a":
            state["steer"] = clamp(state["steer"] - args.keyboard_steer_step, -1.0, 1.0)
        elif key == "d":
            state["steer"] = clamp(state["steer"] + args.keyboard_steer_step, -1.0, 1.0)
        elif key == "c":
            state["throttle"] = 0.0
            state["brake"] = 0.0
            state["steer"] = 0.0
        elif key in {" ", "x"}:
            state["throttle"] = 0.0
            state["brake"] = 1.0
        elif key == "r":
            state["reverse"] = not state["reverse"]
            state["throttle"] = 0.0
            state["brake"] = 0.0
            print(f"reverse={state['reverse']}", flush=True)
    return quit_requested


def make_control(state):
    return carla.VehicleControl(
        throttle=clamp(state["throttle"], 0.0, 1.0),
        steer=clamp(state["steer"], -1.0, 1.0),
        brake=clamp(state["brake"], 0.0, 1.0),
        reverse=bool(state["reverse"]),
    )


def main():
    args = parse_args()
    global carla
    import carla as carla_module

    carla = carla_module
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
                "reverse",
            ],
        )
        writer.writeheader()

    start = time.monotonic()
    frames = 0
    control_state = {
        "throttle": clamp(args.throttle, 0.0, 1.0),
        "steer": clamp(args.steer, -1.0, 1.0),
        "brake": clamp(args.brake, 0.0, 1.0),
        "reverse": False,
    }
    if args.keyboard:
        print_keyboard_help()

    try:
        with raw_terminal(args.keyboard):
            while True:
                elapsed = time.monotonic() - start
                if args.duration > 0.0 and elapsed >= args.duration:
                    break

                if args.keyboard and apply_keyboard_input(args, control_state):
                    break

                control = make_control(control_state)
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
                            "reverse": control.reverse,
                        }
                    )

                if args.print_every > 0 and frames % args.print_every == 0:
                    print(
                        f"frame={snapshot.frame} elapsed={elapsed:.2f}s "
                        f"speed={speed:.2f}m/s "
                        f"control=(thr={control.throttle:.2f}, steer={control.steer:.2f}, "
                        f"brake={control.brake:.2f}, reverse={control.reverse}) "
                        f"loc=({transform.location.x:.2f}, {transform.location.y:.2f}, "
                        f"{transform.location.z:.2f})",
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

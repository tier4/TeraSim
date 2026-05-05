#!/usr/bin/env bash
set -euo pipefail

CARLA_CONTAINER=${CARLA_CONTAINER:-carla-novnc-test}
CARLA_HOST=${CARLA_HOST:-127.0.0.1}
CARLA_PORT=${CARLA_PORT:-2000}
ROLE_NAME=${ROLE_NAME:-AV}
DURATION=${DURATION:-30}
SAMPLE_HZ=${SAMPLE_HZ:-20}
WAIT_FOR_TICK=${WAIT_FOR_TICK:-1}
BACKWARD_THRESHOLD=${BACKWARD_THRESHOLD:--0.05}
MIN_MOVEMENT=${MIN_MOVEMENT:-0.01}
PRINT_EVERY=${PRINT_EVERY:-5}
DRAW_TRAIL=${DRAW_TRAIL:-0}
OUTPUT_CSV=${OUTPUT_CSV:-outputs/carla_actor_motion_$(date +%Y%m%d_%H%M%S).csv}

mkdir -p "$(dirname "${OUTPUT_CSV}")"
if ! touch "${OUTPUT_CSV}" 2>/dev/null; then
  cat >&2 <<EOF
ERROR: Cannot write OUTPUT_CSV: ${OUTPUT_CSV}

The output directory or file is probably owned by root from a previous Docker run.
Fix it on the host, then rerun this script:

  sudo chown -R $(id -u):$(id -g) "$(dirname "${OUTPUT_CSV}")"

EOF
  exit 1
fi

echo "Recording CARLA actor motion"
echo "  container:          ${CARLA_CONTAINER}"
echo "  host:               ${CARLA_HOST}:${CARLA_PORT}"
echo "  role_name:          ${ROLE_NAME}"
echo "  duration:           ${DURATION}s"
echo "  wait_for_tick:      ${WAIT_FOR_TICK}"
echo "  backward_threshold: ${BACKWARD_THRESHOLD}m"
echo "  output_csv:         ${OUTPUT_CSV}"
echo

docker exec -i \
  -e CARLA_HOST="${CARLA_HOST}" \
  -e CARLA_PORT="${CARLA_PORT}" \
  -e ROLE_NAME="${ROLE_NAME}" \
  -e DURATION="${DURATION}" \
  -e SAMPLE_HZ="${SAMPLE_HZ}" \
  -e WAIT_FOR_TICK="${WAIT_FOR_TICK}" \
  -e BACKWARD_THRESHOLD="${BACKWARD_THRESHOLD}" \
  -e MIN_MOVEMENT="${MIN_MOVEMENT}" \
  -e PRINT_EVERY="${PRINT_EVERY}" \
  -e DRAW_TRAIL="${DRAW_TRAIL}" \
  "${CARLA_CONTAINER}" \
  python3.10 - > "${OUTPUT_CSV}" <<'PY'
import csv
import math
import os
import sys
import time
from collections import defaultdict

import carla


FIELDNAMES = [
    "wall_time",
    "sim_time",
    "frame",
    "actor_id",
    "role_name",
    "type_id",
    "x",
    "y",
    "z",
    "yaw",
    "pitch",
    "roll",
    "dx",
    "dy",
    "dz",
    "dt",
    "step_distance",
    "signed_forward_delta",
    "estimated_forward_speed",
    "speed_api_x",
    "speed_api_y",
    "speed_api_z",
    "heading_error_deg",
    "backward",
]


def env_float(name, default):
    value = os.environ.get(name)
    return default if value is None or value == "" else float(value)


def env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def role_filter(raw):
    value = raw.strip()
    if value == "" or value.lower() in {"*", "all"}:
        return None
    return {item.strip() for item in value.split(",") if item.strip()}


def angle_delta_deg(a, b):
    return math.degrees(math.atan2(math.sin(math.radians(a - b)), math.cos(math.radians(a - b))))


def selected_vehicles(world, wanted_roles):
    vehicles = list(world.get_actors().filter("vehicle.*"))
    if wanted_roles is None:
        return vehicles
    return [
        actor
        for actor in vehicles
        if actor.attributes.get("role_name", "") in wanted_roles or str(actor.id) in wanted_roles
    ]


def available_roles(world, limit=12):
    rows = []
    for actor in world.get_actors().filter("vehicle.*"):
        rows.append(f"{actor.attributes.get('role_name', '<empty>')}[{actor.id}]")
    return ", ".join(rows[:limit]) if rows else "<none>"


def write_row(writer, row):
    writer.writerow(row)
    sys.stdout.flush()


def main():
    host = os.environ.get("CARLA_HOST", "127.0.0.1")
    port = int(os.environ.get("CARLA_PORT", "2000"))
    wanted_roles = role_filter(os.environ.get("ROLE_NAME", "AV"))
    duration = env_float("DURATION", 30.0)
    sample_hz = max(env_float("SAMPLE_HZ", 20.0), 0.1)
    wait_for_tick = env_bool("WAIT_FOR_TICK", True)
    backward_threshold = env_float("BACKWARD_THRESHOLD", -0.05)
    min_movement = env_float("MIN_MOVEMENT", 0.01)
    print_every = env_float("PRINT_EVERY", 5.0)
    draw_trail = env_bool("DRAW_TRAIL", False)

    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()

    print(f"Connected to CARLA map: {world.get_map().name}", file=sys.stderr, flush=True)
    print(
        "World settings: "
        f"synchronous_mode={settings.synchronous_mode} "
        f"fixed_delta_seconds={settings.fixed_delta_seconds}",
        file=sys.stderr,
        flush=True,
    )

    writer = csv.DictWriter(sys.stdout, fieldnames=FIELDNAMES, lineterminator="\n")
    writer.writeheader()
    sys.stdout.flush()

    last_by_key = {}
    stats = defaultdict(
        lambda: {
            "samples": 0,
            "moving_samples": 0,
            "total_distance": 0.0,
            "total_signed_forward_delta": 0.0,
            "backward_events": 0,
            "min_signed_forward_delta": None,
            "max_step_distance": 0.0,
            "first_backward_events": [],
        }
    )

    deadline = time.monotonic() + duration if duration > 0 else None
    interval = 1.0 / sample_hz
    last_wait_log = 0.0
    last_progress = time.monotonic()

    while deadline is None or time.monotonic() < deadline:
        if wait_for_tick:
            try:
                snapshot = world.wait_for_tick(2.0)
            except RuntimeError:
                time.sleep(interval)
                snapshot = world.get_snapshot()
        else:
            time.sleep(interval)
            snapshot = world.get_snapshot()

        wall_time = time.time()
        sim_time = snapshot.timestamp.elapsed_seconds
        frame = snapshot.frame

        actors = selected_vehicles(world, wanted_roles)
        if not actors:
            now = time.monotonic()
            if now - last_wait_log > 2.0:
                wanted = "all vehicles" if wanted_roles is None else ", ".join(sorted(wanted_roles))
                print(
                    f"No matching vehicle for {wanted}. Available: {available_roles(world)}",
                    file=sys.stderr,
                    flush=True,
                )
                last_wait_log = now
            continue

        for actor in actors:
            role_name = actor.attributes.get("role_name", "")
            key = role_name or str(actor.id)
            transform = actor.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = actor.get_velocity()

            previous = last_by_key.get(key)
            if previous is not None and previous["actor_id"] != actor.id:
                previous = None
            if previous is not None and previous["frame"] == frame:
                continue

            dx = dy = dz = dt = step_distance = signed_forward_delta = estimated_speed = ""
            heading_error = ""
            backward = 0

            if previous is not None:
                dx_value = location.x - previous["x"]
                dy_value = location.y - previous["y"]
                dz_value = location.z - previous["z"]
                dt_value = sim_time - previous["sim_time"]
                if dt_value <= 0:
                    dt_value = wall_time - previous["wall_time"]

                distance_value = math.sqrt(dx_value * dx_value + dy_value * dy_value + dz_value * dz_value)
                previous_yaw_rad = math.radians(previous["yaw"])
                signed_value = dx_value * math.cos(previous_yaw_rad) + dy_value * math.sin(previous_yaw_rad)
                estimated_value = signed_value / dt_value if dt_value > 0 else 0.0

                if distance_value > min_movement:
                    travel_yaw = math.degrees(math.atan2(dy_value, dx_value))
                    heading_error = angle_delta_deg(travel_yaw, previous["yaw"])
                    if signed_value < backward_threshold:
                        backward = 1

                dx = dx_value
                dy = dy_value
                dz = dz_value
                dt = dt_value
                step_distance = distance_value
                signed_forward_delta = signed_value
                estimated_speed = estimated_value

                actor_stats = stats[key]
                actor_stats["moving_samples"] += int(distance_value > min_movement)
                actor_stats["total_distance"] += distance_value
                actor_stats["total_signed_forward_delta"] += signed_value
                actor_stats["max_step_distance"] = max(actor_stats["max_step_distance"], distance_value)
                current_min = actor_stats["min_signed_forward_delta"]
                actor_stats["min_signed_forward_delta"] = (
                    signed_value if current_min is None else min(current_min, signed_value)
                )
                if backward:
                    actor_stats["backward_events"] += 1
                    if len(actor_stats["first_backward_events"]) < 5:
                        actor_stats["first_backward_events"].append(
                            (sim_time, frame, signed_value, distance_value)
                        )

                if draw_trail and distance_value > min_movement:
                    color = carla.Color(r=255, g=0, b=0) if backward else carla.Color(r=0, g=220, b=80)
                    world.debug.draw_line(
                        carla.Location(previous["x"], previous["y"], previous["z"] + 0.2),
                        carla.Location(location.x, location.y, location.z + 0.2),
                        thickness=0.08,
                        color=color,
                        life_time=10.0,
                    )

            stats[key]["samples"] += 1
            write_row(
                writer,
                {
                    "wall_time": wall_time,
                    "sim_time": sim_time,
                    "frame": frame,
                    "actor_id": actor.id,
                    "role_name": role_name,
                    "type_id": actor.type_id,
                    "x": location.x,
                    "y": location.y,
                    "z": location.z,
                    "yaw": rotation.yaw,
                    "pitch": rotation.pitch,
                    "roll": rotation.roll,
                    "dx": dx,
                    "dy": dy,
                    "dz": dz,
                    "dt": dt,
                    "step_distance": step_distance,
                    "signed_forward_delta": signed_forward_delta,
                    "estimated_forward_speed": estimated_speed,
                    "speed_api_x": velocity.x,
                    "speed_api_y": velocity.y,
                    "speed_api_z": velocity.z,
                    "heading_error_deg": heading_error,
                    "backward": backward,
                },
            )

            last_by_key[key] = {
                "actor_id": actor.id,
                "frame": frame,
                "wall_time": wall_time,
                "sim_time": sim_time,
                "x": location.x,
                "y": location.y,
                "z": location.z,
                "yaw": rotation.yaw,
            }

        now = time.monotonic()
        if print_every > 0 and now - last_progress >= print_every:
            total_samples = sum(item["samples"] for item in stats.values())
            total_backward = sum(item["backward_events"] for item in stats.values())
            print(
                f"Progress: samples={total_samples} tracked={len(stats)} backward_events={total_backward}",
                file=sys.stderr,
                flush=True,
            )
            last_progress = now

    print("Motion summary", file=sys.stderr, flush=True)
    if not stats:
        print("  No actor samples were recorded.", file=sys.stderr, flush=True)
        return 2

    total_backward = 0
    for key in sorted(stats):
        item = stats[key]
        total_backward += item["backward_events"]
        min_signed = item["min_signed_forward_delta"]
        min_signed_text = "n/a" if min_signed is None else f"{min_signed:.4f}m"
        print(
            f"  {key}: samples={item['samples']} moving={item['moving_samples']} "
            f"distance={item['total_distance']:.2f}m signed={item['total_signed_forward_delta']:.2f}m "
            f"backward_events={item['backward_events']} min_signed={min_signed_text} "
            f"max_step={item['max_step_distance']:.2f}m",
            file=sys.stderr,
            flush=True,
        )
        for event in item["first_backward_events"]:
            event_time, event_frame, event_signed, event_distance = event
            print(
                f"    backward sample: sim_time={event_time:.2f}s frame={event_frame} "
                f"signed_delta={event_signed:.4f}m step_distance={event_distance:.4f}m",
                file=sys.stderr,
                flush=True,
            )

    if total_backward == 0:
        print(
            "Verdict: no backward CARLA actor transform steps were detected by the API sampler.",
            file=sys.stderr,
            flush=True,
        )
    else:
        print(
            "Verdict: backward CARLA actor transform steps were detected; inspect rows with backward=1.",
            file=sys.stderr,
            flush=True,
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stopped motion recording.", file=sys.stderr, flush=True)
        raise SystemExit(130)
PY

echo
echo "CSV saved: ${OUTPUT_CSV}"

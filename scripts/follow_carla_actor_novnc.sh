#!/usr/bin/env bash
set -euo pipefail

CARLA_CONTAINER=${CARLA_CONTAINER:-carla-novnc-test}
CARLA_HOST=${CARLA_HOST:-127.0.0.1}
CARLA_PORT=${CARLA_PORT:-2000}
ROLE_NAME=${ROLE_NAME:-AV}
CAMERA_MODE=${CAMERA_MODE:-chase}
FOLLOW_DISTANCE=${FOLLOW_DISTANCE:-12.0}
FOLLOW_HEIGHT=${FOLLOW_HEIGHT:-5.0}
FOLLOW_PITCH=${FOLLOW_PITCH:--18.0}
TOPDOWN_HEIGHT=${TOPDOWN_HEIGHT:-60.0}
UPDATE_INTERVAL=${UPDATE_INTERVAL:-0.05}

echo "Following CARLA actor on noVNC"
echo "  container: ${CARLA_CONTAINER}"
echo "  host:      ${CARLA_HOST}:${CARLA_PORT}"
echo "  role_name: ${ROLE_NAME}"
echo "  mode:      ${CAMERA_MODE}"
echo
echo "Stop with Ctrl-C"

docker exec -i \
  -e CARLA_HOST="${CARLA_HOST}" \
  -e CARLA_PORT="${CARLA_PORT}" \
  -e ROLE_NAME="${ROLE_NAME}" \
  -e CAMERA_MODE="${CAMERA_MODE}" \
  -e FOLLOW_DISTANCE="${FOLLOW_DISTANCE}" \
  -e FOLLOW_HEIGHT="${FOLLOW_HEIGHT}" \
  -e FOLLOW_PITCH="${FOLLOW_PITCH}" \
  -e TOPDOWN_HEIGHT="${TOPDOWN_HEIGHT}" \
  -e UPDATE_INTERVAL="${UPDATE_INTERVAL}" \
  "${CARLA_CONTAINER}" \
  bash -lc 'python3.10 - <<'"'"'PY'"'"'
import math
import os
import sys
import time

import carla


def get_env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    return float(value)


def draw_label(world: carla.World, actor: carla.Actor, text: str) -> None:
    world.debug.draw_string(
        actor.get_location() + carla.Location(z=3.0),
        text,
        draw_shadow=False,
        color=carla.Color(r=255, g=180, b=0),
        life_time=0.1,
        persistent_lines=False,
    )


def build_camera_transform(
    actor_transform: carla.Transform,
    mode: str,
    follow_distance: float,
    follow_height: float,
    follow_pitch: float,
    topdown_height: float,
) -> carla.Transform:
    location = actor_transform.location
    rotation = actor_transform.rotation
    yaw_rad = math.radians(rotation.yaw)
    forward_x = math.cos(yaw_rad)
    forward_y = math.sin(yaw_rad)

    if mode == "topdown":
        camera_location = carla.Location(
            x=location.x,
            y=location.y,
            z=location.z + topdown_height,
        )
        camera_rotation = carla.Rotation(pitch=-90.0, yaw=rotation.yaw, roll=0.0)
    elif mode == "front":
        camera_location = carla.Location(
            x=location.x + 1.5 * forward_x,
            y=location.y + 1.5 * forward_y,
            z=location.z + 1.8,
        )
        camera_rotation = carla.Rotation(
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll=rotation.roll,
        )
    else:
        camera_location = carla.Location(
            x=location.x - follow_distance * forward_x,
            y=location.y - follow_distance * forward_y,
            z=location.z + follow_height,
        )
        camera_rotation = carla.Rotation(pitch=follow_pitch, yaw=rotation.yaw, roll=0.0)

    return carla.Transform(camera_location, camera_rotation)


def main() -> int:
    host = os.environ.get("CARLA_HOST", "127.0.0.1")
    port = int(os.environ.get("CARLA_PORT", "2000"))
    role_name = os.environ.get("ROLE_NAME", "AV")
    mode = os.environ.get("CAMERA_MODE", "chase").strip().lower()
    follow_distance = get_env_float("FOLLOW_DISTANCE", 12.0)
    follow_height = get_env_float("FOLLOW_HEIGHT", 5.0)
    follow_pitch = get_env_float("FOLLOW_PITCH", -18.0)
    topdown_height = get_env_float("TOPDOWN_HEIGHT", 60.0)
    update_interval = get_env_float("UPDATE_INTERVAL", 0.05)

    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()

    print(f"Connected to CARLA map: {world.get_map().name}", flush=True)
    print(f"Waiting for actor with role_name={role_name!r}", flush=True)

    last_actor_id = None
    last_wait_log = 0.0

    while True:
        vehicles = world.get_actors().filter("vehicle.*")
        actor = next((item for item in vehicles if item.attributes.get("role_name") == role_name), None)

        if actor is None:
            now = time.time()
            if now - last_wait_log > 2.0:
                print(f"No actor with role_name={role_name!r} yet...", flush=True)
                last_wait_log = now
            time.sleep(update_interval)
            continue

        if actor.id != last_actor_id:
            print(f"Following actor id={actor.id} type={actor.type_id}", flush=True)
            last_actor_id = actor.id

        draw_label(world, actor, f"{role_name} [{actor.id}]")
        spectator.set_transform(
            build_camera_transform(
                actor.get_transform(),
                mode,
                follow_distance,
                follow_height,
                follow_pitch,
                topdown_height,
            )
        )
        time.sleep(update_interval)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped spectator follow.", flush=True)
        raise SystemExit(0)
    except Exception as exc:
        print(f"Spectator follow failed: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
PY'

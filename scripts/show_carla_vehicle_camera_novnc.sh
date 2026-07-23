#!/usr/bin/env bash
set -euo pipefail

CARLA_CONTAINER=${CARLA_CONTAINER:-carla-novnc-test}
CARLA_HOST=${CARLA_HOST:-127.0.0.1}
CARLA_PORT=${CARLA_PORT:-2000}
CARLA_DISPLAY=${CARLA_DISPLAY:-:1}
ROLE_NAME=${ROLE_NAME:-AV}
CAMERA_PROCESS_MARKER=${CAMERA_PROCESS_MARKER:-terasim-carla-camera-${ROLE_NAME}}

CAMERA_PRESET=${CAMERA_PRESET:-driver}
IMAGE_WIDTH=${IMAGE_WIDTH:-1280}
IMAGE_HEIGHT=${IMAGE_HEIGHT:-720}
WINDOW_WIDTH=${WINDOW_WIDTH:-960}
WINDOW_HEIGHT=${WINDOW_HEIGHT:-540}
WINDOW_X=${WINDOW_X:-30}
WINDOW_Y=${WINDOW_Y:-30}
FOV=${FOV:-90}
SENSOR_TICK=${SENSOR_TICK:-0.0}
ATTACHMENT_TYPE=${ATTACHMENT_TYPE:-auto}
ENABLE_POSTPROCESS=${ENABLE_POSTPROCESS:-1}
MAX_FPS=${MAX_FPS:-60}
RECONNECT_INTERVAL=${RECONNECT_INTERVAL:-1.0}
OVERLAY_POSITION=${OVERLAY_POSITION:-bottom}

# Optional per-axis overrides for the selected preset.
CAMERA_X=${CAMERA_X:-}
CAMERA_Y=${CAMERA_Y:-}
CAMERA_Z=${CAMERA_Z:-}
CAMERA_PITCH=${CAMERA_PITCH:-}
CAMERA_YAW=${CAMERA_YAW:-}
CAMERA_ROLL=${CAMERA_ROLL:-}

SDL_VIDEO_WINDOW_POS=${SDL_VIDEO_WINDOW_POS:-${WINDOW_X},${WINDOW_Y}}

echo "Showing attached CARLA vehicle camera on noVNC"
echo "  container:       ${CARLA_CONTAINER}"
echo "  host:            ${CARLA_HOST}:${CARLA_PORT}"
echo "  display:         ${CARLA_DISPLAY}"
echo "  role_name:       ${ROLE_NAME}"
echo "  process marker:  ${CAMERA_PROCESS_MARKER}"
echo "  camera preset:   ${CAMERA_PRESET}"
echo "  attachment:      ${ATTACHMENT_TYPE}"
echo "  sensor:          ${IMAGE_WIDTH}x${IMAGE_HEIGHT}, fov=${FOV}, tick=${SENSOR_TICK}"
echo "  window:          ${WINDOW_WIDTH}x${WINDOW_HEIGHT} at ${SDL_VIDEO_WINDOW_POS}"
echo
echo "Close the pygame window or press Ctrl-C to stop."

docker exec -i \
  -e CARLA_HOST="${CARLA_HOST}" \
  -e CARLA_PORT="${CARLA_PORT}" \
  -e DISPLAY="${CARLA_DISPLAY}" \
  -e SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-x11}" \
  -e SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}" \
  -e SDL_VIDEO_WINDOW_POS="${SDL_VIDEO_WINDOW_POS}" \
  -e PYGAME_HIDE_SUPPORT_PROMPT=1 \
  -e ROLE_NAME="${ROLE_NAME}" \
  -e CAMERA_PROCESS_MARKER="${CAMERA_PROCESS_MARKER}" \
  -e CAMERA_PRESET="${CAMERA_PRESET}" \
  -e IMAGE_WIDTH="${IMAGE_WIDTH}" \
  -e IMAGE_HEIGHT="${IMAGE_HEIGHT}" \
  -e WINDOW_WIDTH="${WINDOW_WIDTH}" \
  -e WINDOW_HEIGHT="${WINDOW_HEIGHT}" \
  -e FOV="${FOV}" \
  -e SENSOR_TICK="${SENSOR_TICK}" \
  -e ATTACHMENT_TYPE="${ATTACHMENT_TYPE}" \
  -e ENABLE_POSTPROCESS="${ENABLE_POSTPROCESS}" \
  -e MAX_FPS="${MAX_FPS}" \
  -e RECONNECT_INTERVAL="${RECONNECT_INTERVAL}" \
  -e OVERLAY_POSITION="${OVERLAY_POSITION}" \
  -e CAMERA_X="${CAMERA_X}" \
  -e CAMERA_Y="${CAMERA_Y}" \
  -e CAMERA_Z="${CAMERA_Z}" \
  -e CAMERA_PITCH="${CAMERA_PITCH}" \
  -e CAMERA_YAW="${CAMERA_YAW}" \
  -e CAMERA_ROLL="${CAMERA_ROLL}" \
  "${CARLA_CONTAINER}" \
  bash -lc 'exec -a "$CAMERA_PROCESS_MARKER" python3.10 -' <<'PY'
from __future__ import annotations

import os
import queue
import signal
import sys
import time
from dataclasses import dataclass

import carla
import numpy as np
import pygame


def handle_stop_signal(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, handle_stop_signal)


@dataclass(frozen=True)
class CameraPreset:
    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    roll: float


CAMERA_PRESETS = {
    # CARLA attachment transforms are relative to the vehicle.
    # x is forward, y is right, z is up.
    "driver": CameraPreset(0.8, -0.35, 1.65, 0.0, 0.0, 0.0),
    "center": CameraPreset(0.8, 0.0, 1.65, 0.0, 0.0, 0.0),
    "hood": CameraPreset(2.2, 0.0, 1.15, -3.0, 0.0, 0.0),
    "bumper": CameraPreset(2.7, 0.0, 0.75, -2.0, 0.0, 0.0),
    "roof": CameraPreset(0.0, 0.0, 2.45, -6.0, 0.0, 0.0),
    "chase": CameraPreset(-5.5, 0.0, 2.2, 6.0, 0.0, 0.0),
    "rear_chase": CameraPreset(-6.5, 0.0, 2.6, 6.0, 0.0, 0.0),
    "third_person": CameraPreset(-6.5, 0.0, 2.6, 6.0, 0.0, 0.0),
    "rear": CameraPreset(-1.2, 0.0, 1.55, 0.0, 180.0, 0.0),
}


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)


def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def build_camera_transform() -> carla.Transform:
    preset_name = os.environ.get("CAMERA_PRESET", "driver").strip().lower()
    if preset_name not in CAMERA_PRESETS:
        valid = ", ".join(sorted(CAMERA_PRESETS))
        raise ValueError(f"Unknown CAMERA_PRESET={preset_name!r}. Valid presets: {valid}")

    preset = CAMERA_PRESETS[preset_name]
    x = env_float("CAMERA_X", preset.x)
    y = env_float("CAMERA_Y", preset.y)
    z = env_float("CAMERA_Z", preset.z)
    pitch = env_float("CAMERA_PITCH", preset.pitch)
    yaw = env_float("CAMERA_YAW", preset.yaw)
    roll = env_float("CAMERA_ROLL", preset.roll)

    return carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
    )


def get_attachment_type() -> carla.AttachmentType:
    raw_name = os.environ.get("ATTACHMENT_TYPE", "Rigid").strip()
    normalized = raw_name.replace("_", "").replace("-", "").lower()
    if normalized == "auto":
        preset_name = os.environ.get("CAMERA_PRESET", "driver").strip().lower()
        if preset_name in {"chase", "rear_chase", "third_person"}:
            return carla.AttachmentType.SpringArmGhost
        return carla.AttachmentType.Rigid

    mapping = {
        "rigid": carla.AttachmentType.Rigid,
        "springarm": carla.AttachmentType.SpringArm,
        "springarmghost": carla.AttachmentType.SpringArmGhost,
    }
    if normalized not in mapping:
        valid = "auto, Rigid, SpringArm, SpringArmGhost"
        raise ValueError(f"Unknown ATTACHMENT_TYPE={raw_name!r}. Valid values: {valid}")
    return mapping[normalized]


def find_vehicle_by_role(world: carla.World, role_name: str) -> carla.Actor | None:
    vehicles = world.get_actors().filter("vehicle.*")
    for actor in vehicles:
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def describe_vehicle_roles(world: carla.World, limit: int = 12) -> str:
    roles = []
    for actor in world.get_actors().filter("vehicle.*"):
        role = actor.attributes.get("role_name", "")
        if role:
            roles.append(f"{role}[{actor.id}]")
    if not roles:
        return "no vehicle role_name attributes found"
    roles = sorted(roles)
    suffix = "" if len(roles) <= limit else f", ... +{len(roles) - limit} more"
    return ", ".join(roles[:limit]) + suffix


def configure_camera_blueprint(world: carla.World) -> carla.ActorBlueprint:
    blueprint = world.get_blueprint_library().find("sensor.camera.rgb")

    attributes = {
        "image_size_x": str(env_int("IMAGE_WIDTH", 1280)),
        "image_size_y": str(env_int("IMAGE_HEIGHT", 720)),
        "fov": str(env_float("FOV", 90.0)),
        "sensor_tick": str(env_float("SENSOR_TICK", 0.0)),
        "enable_postprocess_effects": "true" if env_bool("ENABLE_POSTPROCESS", True) else "false",
    }
    for name, value in attributes.items():
        if blueprint.has_attribute(name):
            blueprint.set_attribute(name, value)
    return blueprint


def drop_oldest_and_put(image_queue: queue.Queue, item: tuple) -> None:
    try:
        image_queue.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        image_queue.get_nowait()
    except queue.Empty:
        pass

    try:
        image_queue.put_nowait(item)
    except queue.Full:
        pass


def drain_queue(image_queue: queue.Queue) -> tuple | None:
    latest = None
    while True:
        try:
            latest = image_queue.get_nowait()
        except queue.Empty:
            return latest


def image_to_surface(item: tuple) -> tuple[pygame.Surface, int, float]:
    frame, timestamp, width, height, raw = item
    array = raw.reshape((height, width, 4))
    rgb = np.ascontiguousarray(array[:, :, :3][:, :, ::-1])
    surface_array = np.ascontiguousarray(rgb.swapaxes(0, 1))
    surface = pygame.surfarray.make_surface(surface_array)
    return surface, frame, timestamp


def draw_text(screen: pygame.Surface, font: pygame.font.Font, lines: list[str]) -> None:
    if not lines:
        return

    width = screen.get_width()
    overlay_height = 10 + 22 * len(lines)
    overlay_y = 0
    if os.environ.get("OVERLAY_POSITION", "bottom").strip().lower() == "bottom":
        overlay_y = max(0, screen.get_height() - overlay_height)

    overlay = pygame.Surface((width, overlay_height), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 155))
    screen.blit(overlay, (0, overlay_y))

    for index, line in enumerate(lines):
        text = font.render(line, True, (245, 245, 245))
        screen.blit(text, (10, overlay_y + 8 + 22 * index))


def spawn_attached_camera(
    world: carla.World,
    vehicle: carla.Actor,
    image_queue: queue.Queue,
) -> carla.Sensor:
    blueprint = configure_camera_blueprint(world)
    transform = build_camera_transform()
    attachment_type = get_attachment_type()
    camera = world.spawn_actor(
        blueprint,
        transform,
        attach_to=vehicle,
        attachment_type=attachment_type,
    )

    def on_image(image: carla.Image) -> None:
        raw = np.frombuffer(image.raw_data, dtype=np.uint8).copy()
        drop_oldest_and_put(
            image_queue,
            (image.frame, image.timestamp, image.width, image.height, raw),
        )

    camera.listen(on_image)
    return camera


def stop_sensor(sensor: carla.Sensor | None) -> None:
    if sensor is None:
        return
    try:
        sensor.stop()
    except RuntimeError:
        pass
    try:
        sensor.destroy()
    except RuntimeError:
        pass


def main() -> int:
    host = os.environ.get("CARLA_HOST", "127.0.0.1")
    port = env_int("CARLA_PORT", 2000)
    role_name = os.environ.get("ROLE_NAME", "AV")
    reconnect_interval = env_float("RECONNECT_INTERVAL", 1.0)
    max_fps = env_int("MAX_FPS", 60)
    window_width = env_int("WINDOW_WIDTH", 960)
    window_height = env_int("WINDOW_HEIGHT", 540)

    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()

    pygame.display.init()
    pygame.font.init()
    screen = pygame.display.set_mode((window_width, window_height), pygame.RESIZABLE)
    pygame.display.set_caption(f"CARLA {role_name} attached camera")
    font = pygame.font.Font(None, 22)
    clock = pygame.time.Clock()

    print(f"Connected to CARLA map: {world.get_map().name}", flush=True)
    print(f"Waiting for vehicle with role_name={role_name!r}", flush=True)

    image_queue: queue.Queue = queue.Queue(maxsize=1)
    camera: carla.Sensor | None = None
    attached_actor_id: int | None = None
    last_actor_check = 0.0
    last_wait_log = 0.0
    last_frame_time = time.monotonic()
    latest_surface: pygame.Surface | None = None
    latest_frame: int | None = None
    latest_timestamp: float | None = None
    status = "waiting for actor"
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False

            now = time.monotonic()
            if now - last_actor_check >= reconnect_interval:
                last_actor_check = now
                vehicle = find_vehicle_by_role(world, role_name)

                if vehicle is None:
                    if camera is not None:
                        stop_sensor(camera)
                        drain_queue(image_queue)
                        camera = None
                        attached_actor_id = None
                        latest_surface = None
                    status = f"waiting for role_name={role_name!r}"
                    if now - last_wait_log > 2.0:
                        print(f"No vehicle with role_name={role_name!r} yet.", flush=True)
                        print(f"Available roles: {describe_vehicle_roles(world)}", flush=True)
                        last_wait_log = now
                elif attached_actor_id != vehicle.id or camera is None:
                    stop_sensor(camera)
                    drain_queue(image_queue)
                    camera = spawn_attached_camera(world, vehicle, image_queue)
                    attached_actor_id = vehicle.id
                    latest_surface = None
                    status = f"attached to {role_name}[{vehicle.id}] {vehicle.type_id}"
                    print(status, flush=True)

            latest_item = drain_queue(image_queue)
            if latest_item is not None:
                latest_surface, latest_frame, latest_timestamp = image_to_surface(latest_item)
                last_frame_time = now

            if latest_surface is None:
                screen.fill((12, 12, 12))
            else:
                screen_size = screen.get_size()
                if latest_surface.get_size() == screen_size:
                    scaled = latest_surface
                else:
                    scaled = pygame.transform.smoothscale(latest_surface, screen_size)
                screen.blit(scaled, (0, 0))

            age = now - last_frame_time
            frame_text = "no frame yet"
            if latest_frame is not None and latest_timestamp is not None:
                frame_text = f"frame={latest_frame} sim_time={latest_timestamp:.2f}s age={age:.2f}s"
            draw_text(
                screen,
                font,
                [
                    f"{status}  preset={os.environ.get('CAMERA_PRESET', 'driver')}",
                    frame_text,
                    "close window, Esc, or q to stop",
                ],
            )
            pygame.display.flip()
            clock.tick(max_fps)
    finally:
        stop_sensor(camera)
        pygame.quit()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped attached camera viewer.", flush=True)
        raise SystemExit(0)
    except Exception as exc:
        print(f"Attached camera viewer failed: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
PY

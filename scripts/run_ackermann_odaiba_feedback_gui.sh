#!/usr/bin/env bash
set -euo pipefail

CARLA_CONTAINER_NAME=${CARLA_CONTAINER_NAME:-carla-novnc-test}
CARLA_COMPOSE_FILE=${CARLA_COMPOSE_FILE:-docker-compose.carla-novnc.yml}
CARLA_NOVNC_PORT=${CARLA_NOVNC_PORT:-6092}
export CARLA_NOVNC_PORT
CARLA_RENDER_OFFSCREEN=${CARLA_RENDER_OFFSCREEN:-0}
START_CARLA_NOVNC=${START_CARLA_NOVNC:-1}
WAIT_FOR_CARLA_TIMEOUT=${WAIT_FOR_CARLA_TIMEOUT:-180}
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaiba_tl_mapping}
CARLA_PACKAGE_PATH=${CARLA_PACKAGE_PATH:-examples/maps/odaiba_ll2/tlmappings_0708/odaiba_tl_mapping_294096eb1-dirty.tar.gz}
IMPORT_CARLA_PACKAGE=${IMPORT_CARLA_PACKAGE:-auto}
LOAD_CARLA_MAP=${LOAD_CARLA_MAP:-1}
CLEAN_CARLA_ACTORS=${CLEAN_CARLA_ACTORS:-1}
CLEAN_CARLA_ACTORS_ON_EXIT=${CLEAN_CARLA_ACTORS_ON_EXIT:-1}
ATTACH_CHASE_CAMERA=${ATTACH_CHASE_CAMERA:-1}
ATTACH_CAMERA_ROLE_NAME=${ATTACH_CAMERA_ROLE_NAME:-AV}
ATTACH_CAMERA_PRESET=${ATTACH_CAMERA_PRESET:-chase}
ATTACH_CAMERA_LOG=${ATTACH_CAMERA_LOG:-/tmp/terasim_odaiba_chase_camera.log}
CAMERA_PROCESS_MARKER=${CAMERA_PROCESS_MARKER:-terasim-carla-camera-${ATTACH_CAMERA_ROLE_NAME}}
ENABLE_SUMO_GUI=${ENABLE_SUMO_GUI:-1}
SUMO_DISPLAY=${SUMO_DISPLAY:-:20}
SUMO_NOVNC_PORT=${SUMO_NOVNC_PORT:-6093}
SUMO_VNC_PORT=${SUMO_VNC_PORT:-5913}
VNC_PASSWORD=${VNC_PASSWORD:-headless}
SUMO_GUI_TRACK_VEHICLE=${SUMO_GUI_TRACK_VEHICLE:-1}
SUMO_GUI_TRACK_ZOOM=${SUMO_GUI_TRACK_ZOOM:-3000}
SUMO_GUI_REALTIME=${SUMO_GUI_REALTIME:-1}

if [ -z "${CARLA_HOST:-}" ]; then
    CARLA_HOST="${CARLA_CONTAINER_NAME}"
fi

if [ -z "${CARLA_PORT:-}" ]; then
    if [ "${CARLA_HOST}" = "${CARLA_CONTAINER_NAME}" ]; then
        CARLA_PORT=2000
    else
        CARLA_PORT=2010
    fi
fi

CARLA_COSIM_ACKERMANN_FEEDBACK_MODE=${CARLA_COSIM_ACKERMANN_FEEDBACK_MODE:-apply}
CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS=${CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS:-*}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_tlmappings_0708.yaml}
CARLA_COSIM_VEHICLE_CONTROL_MODE=${CARLA_COSIM_VEHICLE_CONTROL_MODE:-ackermann_physics}
CARLA_COSIM_BATCH_TRANSFORM=${CARLA_COSIM_BATCH_TRANSFORM:-1}
CARLA_COSIM_BATCH_SPAWN=${CARLA_COSIM_BATCH_SPAWN:-1}
CARLA_COSIM_ACTOR_FILTER=${CARLA_COSIM_ACTOR_FILTER:-1}
CARLA_COSIM_ACTOR_FILTER_CENTER_ID=${CARLA_COSIM_ACTOR_FILTER_CENTER_ID:-AV}
CARLA_COSIM_ACTOR_FILTER_RADIUS=${CARLA_COSIM_ACTOR_FILTER_RADIUS:-300}
CARLA_COSIM_ACTOR_FILTER_HYSTERESIS=${CARLA_COSIM_ACTOR_FILTER_HYSTERESIS:-20}
TERASIM_COSIM_STATE_FILTER=${TERASIM_COSIM_STATE_FILTER:-${CARLA_COSIM_ACTOR_FILTER}}
TERASIM_COSIM_STATE_FILTER_CENTER_ID=${TERASIM_COSIM_STATE_FILTER_CENTER_ID:-${CARLA_COSIM_ACTOR_FILTER_CENTER_ID}}
TERASIM_COSIM_STATE_FILTER_RADIUS=${TERASIM_COSIM_STATE_FILTER_RADIUS:-320}
CARLA_COSIM_MAX_STEPS=${CARLA_COSIM_MAX_STEPS:-0}

CAMERA_PID=""

is_enabled() {
    case "${1,,}" in
        1|true|yes|on)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

stop_chase_camera() {
    if docker inspect "${CARLA_CONTAINER_NAME}" >/dev/null 2>&1; then
        docker exec "${CARLA_CONTAINER_NAME}" \
            pkill -TERM -f "^${CAMERA_PROCESS_MARKER}([[:space:]]|$)" \
            >/dev/null 2>&1 || true
    fi

    if [ -n "${CAMERA_PID}" ]; then
        kill "${CAMERA_PID}" 2>/dev/null || true
        wait "${CAMERA_PID}" 2>/dev/null || true
        CAMERA_PID=""
    fi
}

cleanup() {
    if is_enabled "${CLEAN_CARLA_ACTORS_ON_EXIT}" \
        && docker inspect "${CARLA_CONTAINER_NAME}" >/dev/null 2>&1; then
        clean_carla_actors || true
    fi
    stop_chase_camera
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_carla() {
    local deadline
    deadline=$((SECONDS + WAIT_FOR_CARLA_TIMEOUT))
    echo "Waiting for CARLA RPC in ${CARLA_CONTAINER_NAME}..."
    while [ "${SECONDS}" -lt "${deadline}" ]; do
        if docker exec -i "${CARLA_CONTAINER_NAME}" python3.10 - <<'PY' >/dev/null 2>&1
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(3.0)
client.get_server_version()
PY
        then
            docker exec -i "${CARLA_CONTAINER_NAME}" python3.10 - <<'PY'
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(10.0)
print("CARLA server version:", client.get_server_version())
PY
            return 0
        fi
        sleep 2
    done
    echo "ERROR: CARLA did not become ready within ${WAIT_FOR_CARLA_TIMEOUT}s." >&2
    return 1
}

map_available() {
    docker exec -i "${CARLA_CONTAINER_NAME}" python3.10 - "${CARLA_PACKAGE_MAP_NAME}" <<'PY' >/dev/null
import sys
import carla

requested = sys.argv[1]
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)

def matches(loaded_name, requested_name):
    return (
        loaded_name == requested_name
        or loaded_name.split("/")[-1] == requested_name
        or loaded_name.endswith("/" + requested_name)
    )

maps = client.get_available_maps()
raise SystemExit(0 if any(matches(name, requested) for name in maps) else 1)
PY
}

import_carla_package_if_needed() {
    if map_available; then
        echo "CARLA map package is already available: ${CARLA_PACKAGE_MAP_NAME}"
        return 0
    fi

    if [ "${IMPORT_CARLA_PACKAGE}" = "auto" ] || is_enabled "${IMPORT_CARLA_PACKAGE}"; then
        if [ ! -f "${CARLA_PACKAGE_PATH}" ]; then
            echo "ERROR: CARLA package not found: ${CARLA_PACKAGE_PATH}" >&2
            return 1
        fi
        echo "Importing CARLA package: ${CARLA_PACKAGE_PATH}"
        docker exec -u root "${CARLA_CONTAINER_NAME}" bash -lc \
            'mkdir -p /workspace/Import && chown -R carla:carla /workspace/Import'
        docker cp "${CARLA_PACKAGE_PATH}" "${CARLA_CONTAINER_NAME}:/workspace/Import/"
        docker exec "${CARLA_CONTAINER_NAME}" bash -lc 'cd /workspace && ./ImportAssets.sh'
        docker compose -f "${CARLA_COMPOSE_FILE}" restart carla_novnc
        wait_for_carla
        return 0
    fi

    echo "ERROR: CARLA map ${CARLA_PACKAGE_MAP_NAME} is not available. Set IMPORT_CARLA_PACKAGE=1 or import it manually." >&2
    return 1
}

load_carla_map() {
    echo "Loading CARLA map if needed: ${CARLA_PACKAGE_MAP_NAME}"
    docker exec -i "${CARLA_CONTAINER_NAME}" python3.10 - "${CARLA_PACKAGE_MAP_NAME}" <<'PY'
import sys
import time
import carla

requested = sys.argv[1]
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(600.0)

def matches(loaded_name, requested_name):
    return (
        loaded_name == requested_name
        or loaded_name.split("/")[-1] == requested_name
        or loaded_name.endswith("/" + requested_name)
    )

world = client.get_world()
current = world.get_map().name
print("Current CARLA map:", current)
if not matches(current, requested):
    print("Loading CARLA map:", requested)
    world = client.load_world(requested)
    time.sleep(5.0)

for _ in range(60):
    world = client.get_world()
    current = world.get_map().name
    if matches(current, requested):
        settings = world.get_settings()
        if settings.no_rendering_mode:
            settings.no_rendering_mode = False
            world.apply_settings(settings)
            print("Disabled CARLA no_rendering_mode for GUI run")
        print("Loaded CARLA map:", current)
        raise SystemExit(0)
    print("Waiting for requested map, current:", current)
    time.sleep(2.0)

print(f"ERROR: requested {requested!r} but current map is {current!r}", file=sys.stderr)
raise SystemExit(1)
PY
}

clean_carla_actors() {
    echo "Removing existing CARLA vehicle and sensor actors..."
    docker exec -i "${CARLA_CONTAINER_NAME}" python3.10 - <<'PY'
import time

import carla

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.get_world()
actors = world.get_actors()
targets = [
    actor
    for actor in actors
    if actor.type_id.startswith("vehicle.") or actor.type_id.startswith("sensor.")
]

vehicle_count = len(actors.filter("vehicle.*"))
sensor_count = len(actors.filter("sensor.*"))
print(f"CARLA actors before cleanup: vehicles={vehicle_count}, sensors={sensor_count}")

if targets:
    responses = client.apply_batch_sync(
        [carla.command.DestroyActor(actor.id) for actor in targets],
        world.get_settings().synchronous_mode,
    )
    errors = [response.error for response in responses if response.error]
    if errors:
        for error in errors:
            print(f"ERROR: failed to destroy CARLA actor: {error}")
        raise SystemExit(1)
    time.sleep(1.0)

actors = world.get_actors()
vehicle_count = len(actors.filter("vehicle.*"))
sensor_count = len(actors.filter("sensor.*"))
print(f"CARLA actors after cleanup: vehicles={vehicle_count}, sensors={sensor_count}")
if vehicle_count or sensor_count:
    raise SystemExit("ERROR: CARLA actor cleanup did not complete")
PY
}

start_chase_camera() {
    stop_chase_camera
    mkdir -p "$(dirname "${ATTACH_CAMERA_LOG}")"
    echo "Starting attached chase camera in CARLA noVNC. Log: ${ATTACH_CAMERA_LOG}"
    CARLA_CONTAINER="${CARLA_CONTAINER_NAME}" \
    CARLA_HOST=127.0.0.1 \
    CARLA_PORT=2000 \
    ROLE_NAME="${ATTACH_CAMERA_ROLE_NAME}" \
    CAMERA_PRESET="${ATTACH_CAMERA_PRESET}" \
    CAMERA_PROCESS_MARKER="${CAMERA_PROCESS_MARKER}" \
    ATTACHMENT_TYPE="${ATTACHMENT_TYPE:-auto}" \
    ./scripts/show_carla_vehicle_camera_novnc.sh >"${ATTACH_CAMERA_LOG}" 2>&1 &
    CAMERA_PID=$!
}

export CARLA_RENDER_OFFSCREEN

if is_enabled "${START_CARLA_NOVNC}"; then
    echo "Starting CARLA noVNC container via ${CARLA_COMPOSE_FILE}..."
    docker compose -f "${CARLA_COMPOSE_FILE}" up -d
fi

wait_for_carla
import_carla_package_if_needed

if is_enabled "${LOAD_CARLA_MAP}"; then
    load_carla_map
fi

if is_enabled "${CLEAN_CARLA_ACTORS}"; then
    clean_carla_actors
fi

if [ -z "${CARLA_DOCKER_NETWORK:-}" ]; then
    CARLA_DOCKER_NETWORK=$(
        docker inspect -f '{{range $name, $net := .NetworkSettings.Networks}}{{$name}}{{end}}' \
            "${CARLA_CONTAINER_NAME}" 2>/dev/null || true
    )
fi
CARLA_DOCKER_NETWORK=${CARLA_DOCKER_NETWORK:-terasim_default}

if is_enabled "${ATTACH_CHASE_CAMERA}"; then
    start_chase_camera
fi

echo "Using CARLA container: ${CARLA_CONTAINER_NAME}"
echo "Using CARLA Docker network: ${CARLA_DOCKER_NETWORK}"
echo "Using CARLA host: ${CARLA_HOST}"
echo "Using CARLA port: ${CARLA_PORT}"
echo "Using CARLA map: ${CARLA_PACKAGE_MAP_NAME}"
echo "CARLA noVNC: http://localhost:${CARLA_NOVNC_PORT}/vnc.html  password: ${VNC_PASSWORD}"
echo "Using CARLA offscreen rendering: ${CARLA_RENDER_OFFSCREEN}"
echo "Using TeraSim config: ${TERASIM_CONFIG}"
echo "Using vehicle control mode: ${CARLA_COSIM_VEHICLE_CONTROL_MODE}"
echo "Using Ackermann feedback: mode=${CARLA_COSIM_ACKERMANN_FEEDBACK_MODE} actors=${CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS}"
echo "Using CARLA batching: transform=${CARLA_COSIM_BATCH_TRANSFORM} spawn=${CARLA_COSIM_BATCH_SPAWN}"
echo "Using CARLA actor filter: enabled=${CARLA_COSIM_ACTOR_FILTER} center=${CARLA_COSIM_ACTOR_FILTER_CENTER_ID} enterRadius=${CARLA_COSIM_ACTOR_FILTER_RADIUS}m hysteresis=${CARLA_COSIM_ACTOR_FILTER_HYSTERESIS}m"
echo "Using TeraSim state filter: enabled=${TERASIM_COSIM_STATE_FILTER} center=${TERASIM_COSIM_STATE_FILTER_CENTER_ID} radius=${TERASIM_COSIM_STATE_FILTER_RADIUS}m"
echo "Using attached chase camera: enabled=${ATTACH_CHASE_CAMERA} role=${ATTACH_CAMERA_ROLE_NAME} preset=${ATTACH_CAMERA_PRESET}"
echo "Using SUMO GUI/noVNC: enabled=${ENABLE_SUMO_GUI} noVNC=${SUMO_NOVNC_PORT} VNC=${SUMO_VNC_PORT} display=${SUMO_DISPLAY} track=${SUMO_GUI_TRACK_VEHICLE} zoom=${SUMO_GUI_TRACK_ZOOM} realtime=${SUMO_GUI_REALTIME}"
if is_enabled "${ENABLE_SUMO_GUI}"; then
    echo "SUMO noVNC: http://localhost:${SUMO_NOVNC_PORT}/vnc.html  password: ${VNC_PASSWORD}"
fi

export CARLA_HOST
export CARLA_PORT
export CARLA_DOCKER_NETWORK
export TERASIM_CONFIG
export CARLA_COSIM_VEHICLE_CONTROL_MODE
export CARLA_COSIM_ACKERMANN_FEEDBACK_MODE
export CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS
export CARLA_COSIM_BATCH_TRANSFORM
export CARLA_COSIM_BATCH_SPAWN
export CARLA_COSIM_ACTOR_FILTER
export CARLA_COSIM_ACTOR_FILTER_CENTER_ID
export CARLA_COSIM_ACTOR_FILTER_RADIUS
export CARLA_COSIM_ACTOR_FILTER_HYSTERESIS
export TERASIM_COSIM_STATE_FILTER
export TERASIM_COSIM_STATE_FILTER_CENTER_ID
export TERASIM_COSIM_STATE_FILTER_RADIUS
export CARLA_COSIM_MAX_STEPS
export ENABLE_SUMO_GUI
export SUMO_DISPLAY
export SUMO_NOVNC_PORT
export SUMO_VNC_PORT
export VNC_PASSWORD
export SUMO_GUI_TRACK_VEHICLE
export SUMO_GUI_TRACK_ZOOM
export SUMO_GUI_REALTIME

docker compose -f docker-compose.ackermann-odaiba-feedback-gui.yml build terasim_ackermann_odaiba_feedback_gui

docker compose -f docker-compose.ackermann-odaiba-feedback-gui.yml run \
    --rm \
    --service-ports \
    --name terasim-odaiba-ackermann-feedback-gui \
    terasim_ackermann_odaiba_feedback_gui

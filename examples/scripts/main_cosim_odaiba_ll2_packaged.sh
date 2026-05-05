#!/bin/bash
set -e
umask 000

MAX_SIM_TIME=${MAX_SIM_TIME:-300}
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaibatest5}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_ll2.yaml}
CARLA_MAP_LOAD_TIMEOUT=${CARLA_MAP_LOAD_TIMEOUT:-600}
CARLA_HOST=${CARLA_HOST:-localhost}
TERASIM_PORT=${TERASIM_PORT:-8000}
CARLA_COSIM_ASYNC_MODE=${CARLA_COSIM_ASYNC_MODE:-0}
ENABLE_SUMO_GUI=${ENABLE_SUMO_GUI:-1}
SUMO_DISPLAY=${SUMO_DISPLAY:-:20}
SUMO_NOVNC_PORT=${SUMO_NOVNC_PORT:-6093}
SUMO_VNC_PORT=${SUMO_VNC_PORT:-5913}
VNC_PASSWORD=${VNC_PASSWORD:-headless}
SUMO_GUI_TRACK_VEHICLE=${SUMO_GUI_TRACK_VEHICLE:-AV}
SUMO_GUI_TRACK_ZOOM=${SUMO_GUI_TRACK_ZOOM:-3000}

ORIGINAL_TERASIM_CONFIG=${TERASIM_CONFIG}
RUNTIME_TERASIM_CONFIG=""
SUMO_NOVNC_PID=""
TERASIM_PID=""

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

cleanup() {
    if [ -n "${TERASIM_PID}" ]; then
        kill "${TERASIM_PID}" 2>/dev/null || true
        wait "${TERASIM_PID}" 2>/dev/null || true
        TERASIM_PID=""
    fi

    if [ -n "${SUMO_NOVNC_PID}" ]; then
        kill "${SUMO_NOVNC_PID}" 2>/dev/null || true
        wait "${SUMO_NOVNC_PID}" 2>/dev/null || true
        SUMO_NOVNC_PID=""
    fi
}
trap cleanup EXIT INT TERM

echo "=========================================="
echo " TeraSim-CARLA Odaiba LL2 Packaged Co-Sim"
echo "  CARLA map: ${CARLA_PACKAGE_MAP_NAME}"
echo "  Max time: ${MAX_SIM_TIME}s"
echo "  Config: ${TERASIM_CONFIG}"
echo "  TeraSim port: ${TERASIM_PORT}"
echo "  SUMO GUI/noVNC: ${ENABLE_SUMO_GUI}"
echo "  SUMO GUI track vehicle: ${SUMO_GUI_TRACK_VEHICLE}"
echo "  SUMO GUI track zoom: ${SUMO_GUI_TRACK_ZOOM}"
echo "=========================================="

if is_enabled "${ENABLE_SUMO_GUI}"; then
    echo "[1/6] Starting SUMO noVNC desktop..."
    export SUMO_DISPLAY
    export SUMO_NOVNC_PORT
    export SUMO_VNC_PORT
    export VNC_PASSWORD
    export SUMO_GUI_TRACK_VEHICLE
    export SUMO_GUI_TRACK_ZOOM
    export DISPLAY="${SUMO_DISPLAY}"
    export USE_LIBSUMO=0

    start_sumo_novnc.sh >/tmp/sumo-novnc.log 2>&1 &
    SUMO_NOVNC_PID=$!
    sleep 1
    if ! kill -0 "${SUMO_NOVNC_PID}" 2>/dev/null; then
        echo "ERROR: SUMO noVNC desktop failed to start."
        cat /tmp/sumo-novnc.log || true
        exit 1
    fi

    RUNTIME_TERASIM_CONFIG="/tmp/$(basename "${TERASIM_CONFIG}" .yaml)_sumo_gui_$$.yaml"
    TERASIM_CONFIG_SOURCE="${TERASIM_CONFIG}" \
    TERASIM_RUNTIME_CONFIG="${RUNTIME_TERASIM_CONFIG}" \
    python3 - <<'PY'
import os
import yaml

source_path = os.environ["TERASIM_CONFIG_SOURCE"]
runtime_path = os.environ["TERASIM_RUNTIME_CONFIG"]

with open(source_path, "r") as file:
    config = yaml.safe_load(file)

simulator = config.setdefault("simulator", {})
parameters = simulator.setdefault("parameters", {})
parameters["gui_flag"] = True

with open(runtime_path, "w") as file:
    yaml.safe_dump(config, file, sort_keys=False)
PY
    TERASIM_CONFIG="${RUNTIME_TERASIM_CONFIG}"

    echo "  SUMO noVNC: http://localhost:${SUMO_NOVNC_PORT}/vnc.html"
    echo "  SUMO VNC password: ${VNC_PASSWORD}"
    echo "  Runtime TeraSim config: ${TERASIM_CONFIG}"
    echo "  Source TeraSim config: ${ORIGINAL_TERASIM_CONFIG}"
else
    echo "[1/6] SUMO GUI/noVNC disabled."
fi

echo "[2/6] Starting Redis server..."
if python3 -c "
import socket
s = socket.socket()
s.settimeout(1.0)
result = s.connect_ex(('127.0.0.1', 6379))
s.close()
raise SystemExit(0 if result == 0 else 1)
" >/dev/null 2>&1; then
    echo "  Redis is already available on localhost:6379. Reusing it."
else
    redis-server &
fi

echo "[3/6] Starting TeraSim server..."
uvicorn terasim_service.api:app --host 0.0.0.0 --port "${TERASIM_PORT}" &
TERASIM_PID=$!

echo "[4/6] Waiting 30s for TeraSim server to be ready..."
sleep 30

echo "[5/6] Checking CARLA server..."
echo "------------------------------------------"

python3 -u -c "
import carla
client = carla.Client('${CARLA_HOST}', ${CARLA_PORT:-2010})
client.set_timeout(5.0)
version = client.get_server_version()
print(f'CARLA server version: {version}')
" >/dev/null

echo "  CARLA is available."

echo "[6/6] Loading packaged CARLA map and starting co-simulation..."
echo "------------------------------------------"

python3 -u -c "
import carla
import os
import sys
import time

carla_host = os.environ.get('CARLA_HOST', '${CARLA_HOST:-localhost}')
carla_port = int(os.environ.get('CARLA_PORT', '${CARLA_PORT:-2010}'))
requested_map = os.environ.get('CARLA_PACKAGE_MAP_NAME', 'odaibatest5')
load_timeout = float(os.environ.get('CARLA_MAP_LOAD_TIMEOUT', '${CARLA_MAP_LOAD_TIMEOUT:-600}'))

def map_matches(loaded_name, requested_name):
    return (
        loaded_name == requested_name
        or loaded_name.split('/')[-1] == requested_name
        or loaded_name.endswith('/' + requested_name)
    )

def get_current_map(timeout):
    probe = carla.Client(carla_host, carla_port)
    probe.set_timeout(timeout)
    return probe.get_world().get_map().name

deadline = time.time() + load_timeout
current_map = None
while time.time() < deadline:
    print(f'Loading packaged CARLA map {requested_map!r}...')
    try:
        current_map = get_current_map(10.0)
        print(f'  Current CARLA map: {current_map}')
        if map_matches(current_map, requested_map):
            print(f'Packaged CARLA map already loaded: {current_map}')
            sys.exit(0)
        break
    except Exception as e:
        print(f'  CARLA map probe not ready yet: {e}')
        time.sleep(5.0)
else:
    print(f'ERROR: Timed out waiting for CARLA to become responsive before map load.')
    sys.exit(1)

client = carla.Client(carla_host, carla_port)
client.set_timeout(load_timeout)

try:
    available_maps = client.get_available_maps()
except Exception as e:
    print(f'Warning: Could not query available maps before load ({e}). Trying load_world directly.')
    available_maps = []

if available_maps and not any(map_matches(map_name, requested_map) for map_name in available_maps):
    print(f'ERROR: Packaged CARLA map {requested_map!r} is not available on the server.')
    print('Import the package in the CARLA container first.')
    print('Available maps:')
    for map_name in available_maps:
        print(f'  - {map_name}')
    sys.exit(1)

print(f'Loading packaged CARLA map {requested_map!r}...')
try:
    client.load_world(requested_map)
except RuntimeError as e:
    print(f'Initial load_world returned error: {e}')
    print('CARLA may still be switching worlds. Waiting and retrying connection...')

loaded_map = None
deadline = time.time() + load_timeout
while time.time() < deadline:
    time.sleep(5.0)
    try:
        loaded_map = get_current_map(30.0)
        print(f'  Current CARLA map: {loaded_map}')
        if map_matches(loaded_map, requested_map):
            break
    except Exception as retry_e:
        print(f'  CARLA not ready yet: {retry_e}')
else:
    print(f'ERROR: Timed out waiting for packaged CARLA map {requested_map!r} to load.')
    sys.exit(1)

if not map_matches(loaded_map, requested_map):
    print(f'ERROR: Requested {requested_map!r} but CARLA loaded {loaded_map!r}')
    sys.exit(1)
print(f'Packaged CARLA map loaded: {loaded_map}')
" 2>&1

echo "  Waiting for CARLA world to stabilize..."
sleep 5

echo "  Config: ${TERASIM_CONFIG}"
echo "  CARLA co-sim async mode: ${CARLA_COSIM_ASYNC_MODE}"
if [ -n "${CARLA_COSIM_MOTION_LOG:-}" ]; then
    echo "  CARLA co-sim motion log: ${CARLA_COSIM_MOTION_LOG}"
    echo "  CARLA co-sim diagnostic roles: ${CARLA_COSIM_DIAG_ROLE_NAMES:-AV}"
fi
if [ -n "${CARLA_COSIM_PROFILE_LOG:-}" ]; then
    echo "  CARLA co-sim profile log: ${CARLA_COSIM_PROFILE_LOG}"
fi
if [ -n "${CARLA_COSIM_ACTOR_PROFILE_LOG:-}" ]; then
    echo "  CARLA co-sim actor profile log: ${CARLA_COSIM_ACTOR_PROFILE_LOG}"
fi
echo "------------------------------------------"

CARLA_COSIM_ARGS=(
    --terasim_config "${TERASIM_CONFIG}"
    --carla_host "${CARLA_HOST:-localhost}"
    --carla_port ${CARLA_PORT:-2010}
    --carla_timeout 600
    --terasim_port ${TERASIM_PORT}
)

if [ "${CARLA_COSIM_ASYNC_MODE}" = "1" ]; then
    CARLA_COSIM_ARGS+=(--async_mode)
fi

CARLA_EXIT=0
python3 /app/examples/scripts/carla_cosim_main_local_alignment.py \
    "${CARLA_COSIM_ARGS[@]}" || CARLA_EXIT=$?

echo "=========================================="
echo " CARLA client exited (code: $CARLA_EXIT)"
echo " Stopping TeraSim server and SUMO noVNC desktop..."
echo "=========================================="

cleanup
trap - EXIT INT TERM

echo "=========================================="
echo " Simulation complete. Container exiting."
echo "=========================================="

#!/bin/bash
set -e
umask 000

MAX_SIM_TIME=${MAX_SIM_TIME:-300}
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaiba_tl_mapping}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_ll2.yaml}
CARLA_MAP_LOAD_TIMEOUT=${CARLA_MAP_LOAD_TIMEOUT:-600}
CARLA_HOST=${CARLA_HOST:-localhost}
TERASIM_PORT=${TERASIM_PORT:-8000}
CARLA_COSIM_ASYNC_MODE=${CARLA_COSIM_ASYNC_MODE:-0}
CARLA_COSIM_STEP_LENGTH=${CARLA_COSIM_STEP_LENGTH:-0.1}
CARLA_COSIM_CONTROL_AV=${CARLA_COSIM_CONTROL_AV:-0}
ENABLE_SUMO_GUI=${ENABLE_SUMO_GUI:-1}
SUMO_DISPLAY=${SUMO_DISPLAY:-:20}
SUMO_NOVNC_PORT=${SUMO_NOVNC_PORT:-6093}
SUMO_VNC_PORT=${SUMO_VNC_PORT:-5913}
VNC_PASSWORD=${VNC_PASSWORD:-headless}
SUMO_GUI_TRACK_VEHICLE=${SUMO_GUI_TRACK_VEHICLE:-AV}
SUMO_GUI_TRACK_ZOOM=${SUMO_GUI_TRACK_ZOOM:-3000}
TERASIM_UVICORN_ACCESS_LOG=${TERASIM_UVICORN_ACCESS_LOG:-1}
TERASIM_REPO_ROOT=${TERASIM_REPO_ROOT:-/app}
ENABLE_ODAIBA_TLS_SYNC=${ENABLE_ODAIBA_TLS_SYNC:-1}
ODAIBA_TLS_MIN_COVERAGE=${ODAIBA_TLS_MIN_COVERAGE:-0.90}
ODAIBA_TLS_OUTPUT_DIR=${ODAIBA_TLS_OUTPUT_DIR:-/tmp/terasim-odaiba-tls}
ODAIBA_TLS_MAPPING_DIR=${ODAIBA_TLS_MAPPING_DIR:-/app/examples/maps/odaiba_ll2/tlmappings}
ODAIBA_TLS_TARGET_NET=${ODAIBA_TLS_TARGET_NET:-/app/examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml}
ODAIBA_TLS_SOURCE_SUMOCFG=${ODAIBA_TLS_SOURCE_SUMOCFG:-/app/examples/maps/odaiba_ll2/simulation.sumocfg}
ODAIBA_TLS_SIGNAL_ID_MAPPING=${ODAIBA_TLS_SIGNAL_ID_MAPPING:-${ODAIBA_TLS_MAPPING_DIR}/signal_id_mapping.json}
ODAIBA_TLS_OPENDRIVE_MAPPING=${ODAIBA_TLS_OPENDRIVE_MAPPING:-}

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
echo "  Odaiba TLS sync: ${ENABLE_ODAIBA_TLS_SYNC}"
echo "=========================================="

if is_enabled "${ENABLE_SUMO_GUI}"; then
    echo "[1/7] Starting SUMO noVNC desktop..."
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
    echo "[1/7] SUMO GUI/noVNC disabled."
fi

if is_enabled "${ENABLE_ODAIBA_TLS_SYNC}"; then
    echo "[2/7] Generating Odaiba TLS linkSignalID mapping..."
    mkdir -p "${ODAIBA_TLS_OUTPUT_DIR}"

    if [ -z "${ODAIBA_TLS_OPENDRIVE_MAPPING}" ]; then
        shopt -s nullglob
        ODAIBA_TLS_MAPPING_CANDIDATES=("${ODAIBA_TLS_MAPPING_DIR}"/odaiba_tl_mapping_*.mapping.json)
        shopt -u nullglob
        if [ "${#ODAIBA_TLS_MAPPING_CANDIDATES[@]}" -ne 1 ]; then
            echo "ERROR: Expected exactly one odaiba_tl_mapping_*.mapping.json in ${ODAIBA_TLS_MAPPING_DIR}."
            printf '  - %s\n' "${ODAIBA_TLS_MAPPING_CANDIDATES[@]}"
            exit 1
        fi
        ODAIBA_TLS_OPENDRIVE_MAPPING="${ODAIBA_TLS_MAPPING_CANDIDATES[0]}"
    fi

    ODAIBA_TLS_OUTPUT_NET="${ODAIBA_TLS_OUTPUT_DIR}/odaiba_osmlike_network3_tls_synced.net.xml"
    ODAIBA_TLS_OUTPUT_SUMOCFG="${ODAIBA_TLS_OUTPUT_DIR}/simulation_tls_synced.sumocfg"
    ODAIBA_TLS_REPORT="${ODAIBA_TLS_OUTPUT_DIR}/tls_linksignal_report.json"

    python3 "${TERASIM_REPO_ROOT}/scripts/generate_tls_linksignal_params.py" \
        --sumo-net "${ODAIBA_TLS_TARGET_NET}" \
        --signal-id-mapping "${ODAIBA_TLS_SIGNAL_ID_MAPPING}" \
        --opendrive-lanelet-mapping "${ODAIBA_TLS_OPENDRIVE_MAPPING}" \
        --sumocfg "${ODAIBA_TLS_SOURCE_SUMOCFG}" \
        --output-net "${ODAIBA_TLS_OUTPUT_NET}" \
        --output-sumocfg "${ODAIBA_TLS_OUTPUT_SUMOCFG}" \
        --report "${ODAIBA_TLS_REPORT}" \
        --min-coverage "${ODAIBA_TLS_MIN_COVERAGE}"

    RUNTIME_TERASIM_CONFIG="/tmp/$(basename "${TERASIM_CONFIG}" .yaml)_tls_sync_$$.yaml"
    TERASIM_CONFIG_SOURCE="${TERASIM_CONFIG}" \
    TERASIM_RUNTIME_CONFIG="${RUNTIME_TERASIM_CONFIG}" \
    RUNTIME_SUMO_NET_FILE="${ODAIBA_TLS_OUTPUT_NET}" \
    RUNTIME_SUMO_CONFIG_FILE="${ODAIBA_TLS_OUTPUT_SUMOCFG}" \
    python3 - <<'PY'
import os
import yaml

source_path = os.environ["TERASIM_CONFIG_SOURCE"]
runtime_path = os.environ["TERASIM_RUNTIME_CONFIG"]
sumo_net_file = os.environ["RUNTIME_SUMO_NET_FILE"]
sumo_config_file = os.environ["RUNTIME_SUMO_CONFIG_FILE"]

with open(source_path, "r") as file:
    config = yaml.safe_load(file)

environment = config.setdefault("environment", {})
env_parameters = environment.setdefault("parameters", {})
env_parameters["sumo_net_file_path"] = sumo_net_file
env_parameters["sumo_cfg_file_path"] = sumo_config_file

input_config = config.setdefault("input", {})
input_config["sumo_net_file"] = sumo_net_file
input_config["sumo_config_file"] = sumo_config_file

with open(runtime_path, "w") as file:
    yaml.safe_dump(config, file, sort_keys=False)
PY
    TERASIM_CONFIG="${RUNTIME_TERASIM_CONFIG}"

    echo "  TLS synced SUMO net: ${ODAIBA_TLS_OUTPUT_NET}"
    echo "  TLS synced SUMO config: ${ODAIBA_TLS_OUTPUT_SUMOCFG}"
    echo "  TLS coverage report: ${ODAIBA_TLS_REPORT}"
    echo "  Runtime TeraSim config: ${TERASIM_CONFIG}"
else
    echo "[2/7] Odaiba TLS linkSignalID generation disabled."
fi

echo "[3/7] Starting Redis server..."
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

echo "[4/7] Starting TeraSim server..."
UVICORN_ARGS=(terasim_service.api:app --host 0.0.0.0 --port "${TERASIM_PORT}")
if ! is_enabled "${TERASIM_UVICORN_ACCESS_LOG}"; then
    UVICORN_ARGS+=(--no-access-log)
fi
uvicorn "${UVICORN_ARGS[@]}" &
TERASIM_PID=$!

echo "[5/7] Waiting 30s for TeraSim server to be ready..."
sleep 30

echo "[6/7] Checking CARLA server..."
echo "------------------------------------------"

python3 -u -c "
import carla
client = carla.Client('${CARLA_HOST}', ${CARLA_PORT:-2010})
client.set_timeout(5.0)
version = client.get_server_version()
print(f'CARLA server version: {version}')
" >/dev/null

echo "  CARLA is available."

echo "[7/7] Loading packaged CARLA map and starting co-simulation..."
echo "------------------------------------------"

python3 -u -c "
import carla
import os
import sys
import time

carla_host = os.environ.get('CARLA_HOST', '${CARLA_HOST:-localhost}')
carla_port = int(os.environ.get('CARLA_PORT', '${CARLA_PORT:-2010}'))
requested_map = os.environ.get('CARLA_PACKAGE_MAP_NAME', 'odaiba_tl_mapping')
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
echo "  CARLA co-sim step length: ${CARLA_COSIM_STEP_LENGTH}"
echo "  CARLA co-sim control AV: ${CARLA_COSIM_CONTROL_AV}"
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
    --step_length "${CARLA_COSIM_STEP_LENGTH}"
)

if [ "${CARLA_COSIM_ASYNC_MODE}" = "1" ]; then
    CARLA_COSIM_ARGS+=(--async_mode)
fi

if is_enabled "${CARLA_COSIM_CONTROL_AV}"; then
    CARLA_COSIM_ARGS+=(--control_av)
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

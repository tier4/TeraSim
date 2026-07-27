#!/usr/bin/env bash
set -euo pipefail

CARLA_HOST=${CARLA_HOST:-carla-novnc-test}
CARLA_PORT=${CARLA_PORT:-2000}
GRPC_PORT=${GRPC_PORT:-8200}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_tlmappings_0708.yaml}
CARLA_COSIM_STEP_LENGTH=${CARLA_COSIM_STEP_LENGTH:-0.1}
CARLA_COSIM_MAP_NAME=${CARLA_COSIM_MAP_NAME:-}
CARLA_COSIM_VEHICLE_CONTROL_MODE=${CARLA_COSIM_VEHICLE_CONTROL_MODE:-ackermann_physics}
ENABLE_SUMO_GUI=${ENABLE_SUMO_GUI:-1}
SUMO_DISPLAY=${SUMO_DISPLAY:-:20}
SUMO_GUI_REALTIME=${SUMO_GUI_REALTIME:-1}

DIRECT_PID=""
SUMO_NOVNC_PID=""
COSIM_PID=""
DIRECT_LOG=/tmp/terasim_direct.log
COSIM_SHUTDOWN_TIMEOUT=${COSIM_SHUTDOWN_TIMEOUT:-60}

is_enabled() {
    case "${1,,}" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

stop_cosim_first() {
    if [[ -z "${COSIM_PID}" ]] || ! kill -0 "${COSIM_PID}" 2>/dev/null; then
        return
    fi

    echo "Stopping CARLA co-simulation client before TeraSim/SUMO..."
    kill -TERM "${COSIM_PID}" 2>/dev/null || true

    local deadline=$((SECONDS + COSIM_SHUTDOWN_TIMEOUT))
    while kill -0 "${COSIM_PID}" 2>/dev/null && [[ "${SECONDS}" -lt "${deadline}" ]]; do
        sleep 0.25
    done

    if kill -0 "${COSIM_PID}" 2>/dev/null; then
        echo "WARNING: CARLA co-simulation cleanup did not finish within" \
            "${COSIM_SHUTDOWN_TIMEOUT}s; still waiting without stopping TeraSim/SUMO." >&2
        wait "${COSIM_PID}" 2>/dev/null || true
    fi

    wait "${COSIM_PID}" 2>/dev/null || true
    COSIM_PID=""
    echo "CARLA co-simulation cleanup completed; stopping TeraSim/SUMO."
}

cleanup() {
    status=$?
    trap - EXIT INT TERM

    stop_cosim_first
    if [[ -n "${DIRECT_PID}" ]]; then
        kill "${DIRECT_PID}" 2>/dev/null || true
        wait "${DIRECT_PID}" 2>/dev/null || true
    fi
    if [[ -n "${SUMO_NOVNC_PID}" ]]; then
        kill "${SUMO_NOVNC_PID}" 2>/dev/null || true
        wait "${SUMO_NOVNC_PID}" 2>/dev/null || true
    fi

    if [[ "${status}" -ne 0 && -f "${DIRECT_LOG}" ]]; then
        echo "Last TeraSim direct-runner log lines:" >&2
        tail -80 "${DIRECT_LOG}" >&2 || true
    fi
    exit "${status}"
}
trap cleanup EXIT INT TERM

export USE_LIBSUMO=0
export DISPLAY="${SUMO_DISPLAY}"

if is_enabled "${ENABLE_SUMO_GUI}"; then
    echo "[1/4] Starting SUMO desktop/noVNC on port ${SUMO_NOVNC_PORT:-6093}..."
    /usr/local/bin/start_sumo_novnc >/tmp/sumo-novnc.log 2>&1 &
    SUMO_NOVNC_PID=$!
    sleep 1
    if ! kill -0 "${SUMO_NOVNC_PID}" 2>/dev/null; then
        cat /tmp/sumo-novnc.log >&2 || true
        exit 1
    fi

    RUNTIME_TERASIM_CONFIG="/tmp/$(basename "${TERASIM_CONFIG}" .yaml)_sumo_gui.yaml"
    TERASIM_CONFIG_SOURCE="${TERASIM_CONFIG}" \
    TERASIM_RUNTIME_CONFIG="${RUNTIME_TERASIM_CONFIG}" \
    python3 - <<'PY'
import os
import yaml

with open(os.environ["TERASIM_CONFIG_SOURCE"], encoding="utf-8") as source:
    config = yaml.safe_load(source)
parameters = config.setdefault("simulator", {}).setdefault("parameters", {})
parameters["gui_flag"] = True
parameters["realtime_flag"] = os.environ.get("SUMO_GUI_REALTIME", "1").lower() in {
    "1", "true", "yes", "on"
}
with open(os.environ["TERASIM_RUNTIME_CONFIG"], "w", encoding="utf-8") as output:
    yaml.safe_dump(config, output, sort_keys=False)
PY
    TERASIM_CONFIG="${RUNTIME_TERASIM_CONFIG}"
fi

echo "[2/4] Checking CARLA ${CARLA_HOST}:${CARLA_PORT}..."
python3 -u -c "import carla; c=carla.Client('${CARLA_HOST}', int('${CARLA_PORT}')); c.set_timeout(30); print('CARLA server version:', c.get_server_version())"

echo "[3/4] Starting TeraSim direct gRPC runner on 127.0.0.1:${GRPC_PORT}..."
python3 -m terasim_service.run_direct \
    --config "${TERASIM_CONFIG}" \
    --grpc_port "${GRPC_PORT}" >"${DIRECT_LOG}" 2>&1 &
DIRECT_PID=$!

CARLA_COSIM_ARGS=(
    --terasim_config "${TERASIM_CONFIG}"
    --carla_host "${CARLA_HOST}"
    --carla_port "${CARLA_PORT}"
    --carla_timeout 600
    --direct_addr "127.0.0.1:${GRPC_PORT}"
    --step_length "${CARLA_COSIM_STEP_LENGTH}"
    --vehicle_control_mode "${CARLA_COSIM_VEHICLE_CONTROL_MODE}"
)
if [[ -n "${CARLA_COSIM_MAP_NAME}" ]]; then
    CARLA_COSIM_ARGS+=(--map_name "${CARLA_COSIM_MAP_NAME}")
fi

echo "[4/4] Starting synchronous Ackermann feedback co-simulation..."
echo "  feedback=${CARLA_COSIM_ACKERMANN_FEEDBACK_MODE:-off} actors=${CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS:-}"
echo "  SUMO noVNC: http://localhost:${SUMO_NOVNC_PORT:-6093}/vnc.html"
python3 /app/examples/scripts/carla_cosim_main.py "${CARLA_COSIM_ARGS[@]}" &
COSIM_PID=$!
wait "${COSIM_PID}"
COSIM_STATUS=$?
COSIM_PID=""
exit "${COSIM_STATUS}"

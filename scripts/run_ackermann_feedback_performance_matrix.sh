#!/usr/bin/env bash
set -euo pipefail
PERF_RADII=${PERF_RADII:-20,40,60,80,100}
PERF_RHI_MODES=${PERF_RHI_MODES:-normal,nullrhi}
PERF_REPEATS=${PERF_REPEATS:-1}
PERF_WARMUP_STEPS=${PERF_WARMUP_STEPS:-600}
PERF_MEASUREMENT_STEPS=${PERF_MEASUREMENT_STEPS:-100}
PERF_STEP_LENGTH=${PERF_STEP_LENGTH:-0.05}
PERF_SUMO_THREADS=${PERF_SUMO_THREADS:-8}
PERF_COSIM_TRANSPORT=${PERF_COSIM_TRANSPORT:-grpc}
PERF_OUTPUT_ROOT=${PERF_OUTPUT_ROOT:-outputs/ackermann_feedback_performance_filtered_r300_nofcd_threads8}
PERF_ASSET_ROOT=${PERF_ASSET_ROOT:-/home/h-kawai/TeraSim/examples}
PERF_CARLA_IMAGE=${PERF_CARLA_IMAGE:-carla-novnc:odaiba-perf-base}
PERF_TERASIM_IMAGE=${PERF_TERASIM_IMAGE:-terasim-service:ackermann-feedback-perf}
PERF_CARLA_MAP=${PERF_CARLA_MAP:-odaiba_tl_mapping}
PERF_NETWORK=${PERF_NETWORK:-terasim_default}
PERF_SUMO_STATE_FILE=${PERF_SUMO_STATE_FILE:-}
CARLA_CONTAINER=carla-ackermann-perf
ACTIVE_TERASIM=""
cleanup_condition() {
    if [[ -n "${ACTIVE_TERASIM}" ]]; then
        docker stop -t 90 "${ACTIVE_TERASIM}" >/dev/null 2>&1 || true
        docker rm -f "${ACTIVE_TERASIM}" >/dev/null 2>&1 || true
        ACTIVE_TERASIM=""
    fi
    docker stop -t 30 "${CARLA_CONTAINER}" >/dev/null 2>&1 || true
    docker rm -f "${CARLA_CONTAINER}" >/dev/null 2>&1 || true
}
trap cleanup_condition EXIT INT TERM
if [[ ! -d "${PERF_ASSET_ROOT}" ]]; then echo "ERROR: asset root not found: ${PERF_ASSET_ROOT}" >&2; exit 1; fi
mkdir -p "${PERF_OUTPUT_ROOT}"
PERF_OUTPUT_ROOT=$(realpath "${PERF_OUTPUT_ROOT}")
WORKTREE_OUTPUT=$(realpath outputs)
case "${PERF_OUTPUT_ROOT}/" in "${WORKTREE_OUTPUT}/"*) ;; *) echo "ERROR: PERF_OUTPUT_ROOT must be under ${WORKTREE_OUTPUT}" >&2; exit 1 ;; esac
OUTPUT_REL=${PERF_OUTPUT_ROOT#"${WORKTREE_OUTPUT}/"}
CONTAINER_OUTPUT_ROOT=/app/outputs/${OUTPUT_REL}
IFS=',' read -r -a RADII <<<"${PERF_RADII}"
IFS=',' read -r -a RHI_MODES <<<"${PERF_RHI_MODES}"
TOTAL_STEPS=$((PERF_WARMUP_STEPS + PERF_MEASUREMENT_STEPS))
if ! docker network inspect "${PERF_NETWORK}" >/dev/null 2>&1; then docker network create "${PERF_NETWORK}" >/dev/null; fi

echo "Building TeraSim benchmark image once..."
TERASIM_GUI_IMAGE="${PERF_TERASIM_IMAGE}" docker compose -f docker-compose.ackermann-odaiba-feedback-gui.yml build terasim_ackermann_odaiba_feedback_gui
BASE_CONFIG=/app/examples/scenarios/cosim_odaiba_tlmappings_0708.yaml
NET_FILE=/app/examples/maps/odaiba_ll2/tlmappings_0708/network.net.xml
ROUTE_FILE=/app/examples/maps/odaiba_ll2/tlmappings_0708/period_0p2_filter_check/vehicles.filtered_r300.rou.xml

docker run --rm --entrypoint python -u "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "${PERF_ASSET_ROOT}:/app/examples:ro" -v "${PWD}/scripts:/app/scripts:ro" -v "${WORKTREE_OUTPUT}:/app/outputs" \
    "${PERF_TERASIM_IMAGE}" /app/scripts/prepare_ackermann_feedback_performance.py \
    --base-config "${BASE_CONFIG}" --output-dir "${CONTAINER_OUTPUT_ROOT}" --container-output-dir "${CONTAINER_OUTPUT_ROOT}" \
    --net-file "${NET_FILE}" --route-file "${ROUTE_FILE}" --step-length "${PERF_STEP_LENGTH}" --cache-time 500 \
    --sumo-threads "${PERF_SUMO_THREADS}"
STATE_HOST=${PERF_OUTPUT_ROOT}/sumo_state_500.xml.gz
if [[ -n "${PERF_SUMO_STATE_FILE}" ]]; then
    cp "${PERF_SUMO_STATE_FILE}" "${STATE_HOST}"
elif [[ ! -s "${STATE_HOST}" ]]; then
    echo "Generating deterministic SUMO state at t=500s..."
    docker run --rm --entrypoint sumo -u "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "${PERF_ASSET_ROOT}:/app/examples:ro" -v "${WORKTREE_OUTPUT}:/app/outputs" "${PERF_TERASIM_IMAGE}" \
        -c "${CONTAINER_OUTPUT_ROOT}/sumo_cache_generation.sumocfg" --save-state.times 500 \
        --save-state.files "${CONTAINER_OUTPUT_ROOT}/sumo_state_500.xml.gz" --no-step-log true
fi
wait_for_carla() {
    local deadline=$((SECONDS + 180))
    while [[ "${SECONDS}" -lt "${deadline}" ]]; do
        if docker exec "${CARLA_CONTAINER}" python3.10 -c 'import carla; c=carla.Client("127.0.0.1",2000); c.set_timeout(3); c.get_server_version()' >/dev/null 2>&1; then return 0; fi
        sleep 2
    done
    echo "ERROR: dedicated CARLA did not become ready" >&2
    return 1
}
load_carla_map() {
    docker exec -i "${CARLA_CONTAINER}" python3.10 - "${PERF_CARLA_MAP}" <<'PY'
import sys, time, carla
requested = sys.argv[1]
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(600.0)
def matches(name): return name == requested or name.split("/")[-1] == requested or name.endswith("/" + requested)
world = client.get_world()
if not matches(world.get_map().name):
    client.load_world(requested)
    time.sleep(3)
print("CARLA map:", client.get_world().get_map().name)
PY
}
start_carla() {
    local rhi=$1 null_rhi=0
    [[ "${rhi}" == "nullrhi" ]] && null_rhi=1
    cleanup_condition
    docker run -d --name "${CARLA_CONTAINER}" --runtime=nvidia --gpus all --privileged --shm-size=8g --network "${PERF_NETWORK}" \
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json -e CARLA_NULL_RHI="${null_rhi}" \
        -e CARLA_RENDER_OFFSCREEN=0 -e VNC_PASSWORD=headless -e DISPLAY=:1 \
        -e RESOLUTION=1600x900x24 -e RESX=1600 -e RESY=900 \
        -v "${PWD}/scripts/start_carla_novnc.sh:/start_carla_novnc.sh:ro" \
        -v /usr/share/vulkan/icd.d/nvidia_icd.json:/usr/share/vulkan/icd.d/nvidia_icd.json:ro \
        -v /usr/share/vulkan/implicit_layer.d/nvidia_layers.json:/usr/share/vulkan/implicit_layer.d/nvidia_layers.json:ro \
        "${PERF_CARLA_IMAGE}" >/dev/null
    wait_for_carla
    load_carla_map
}
for rhi in "${RHI_MODES[@]}"; do
    for radius in "${RADII[@]}"; do
        for repeat in $(seq 1 "${PERF_REPEATS}"); do
            effective_radius="${radius}"
            if [[ "${radius}" == "0" ]]; then
                # A core radius of exactly zero disables radius filtering. Use a
                # positive epsilon so the benchmark's 0m condition means AV-only.
                effective_radius="0.000001"
            fi
            condition="rhi-${rhi}_radius-${radius}m_repeat-${repeat}"
            condition_dir="${PERF_OUTPUT_ROOT}/${condition}"
            mkdir -p "${condition_dir}"
            rm -f "${condition_dir}/carla_profile.jsonl" "${condition_dir}/terasim_profile.jsonl"
            python3 - "${condition_dir}/manifest.json" "${rhi}" "${radius}" "${repeat}" "${PERF_WARMUP_STEPS}" "${PERF_MEASUREMENT_STEPS}" "${PERF_SUMO_THREADS}" "${ROUTE_FILE}" "${PERF_COSIM_TRANSPORT}" <<'PY'
import json, sys
path, rhi, radius, repeat, warmup, measured, sumo_threads, route_file, transport = sys.argv[1:]
with open(path, "w", encoding="utf-8") as f:
    json.dump({"rhi": rhi, "radius_m": float(radius), "repeat": int(repeat), "warmup_steps": int(warmup),
               "measurement_steps": int(measured), "step_length_s": 0.05, "period_s": 0.2,
               "sumo_threads": int(sumo_threads), "fcd_output": False, "transport": transport,
               "route_file": route_file}, f, indent=2)
PY
            echo "[${condition}] Starting dedicated CARLA..."
            start_carla "${rhi}"
            echo "[${condition}] Running ${TOTAL_STEPS} steps (${PERF_WARMUP_STEPS} warmup + ${PERF_MEASUREMENT_STEPS} measured)..."
            ACTIVE_TERASIM="terasim-perf-${rhi}-${radius}-${repeat}"
            export TERASIM_GUI_IMAGE="${PERF_TERASIM_IMAGE}" TERASIM_EXAMPLES_HOST_DIR="${PERF_ASSET_ROOT}"
            export CARLA_DOCKER_NETWORK="${PERF_NETWORK}" CARLA_HOST="${CARLA_CONTAINER}" CARLA_PORT=2000
            export TERASIM_CONFIG="${CONTAINER_OUTPUT_ROOT}/cosim_odaiba_period_0p2_cached_t500.yaml"
            export CARLA_COSIM_STEP_LENGTH="${PERF_STEP_LENGTH}" COSIM_EXPECTED_STEP_LENGTH="${PERF_STEP_LENGTH}" CARLA_COSIM_MAX_STEPS="${TOTAL_STEPS}"
            export COSIM_TRANSPORT="${PERF_COSIM_TRANSPORT}"
            export CARLA_COSIM_ACKERMANN_FEEDBACK_MODE=apply CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS="*,AV" CARLA_COSIM_ACKERMANN_FEEDBACK_POSITION_MODE=moveTo
            export CARLA_COSIM_BATCH_TRANSFORM=1 CARLA_COSIM_BATCH_SPAWN=1
            export CARLA_COSIM_ACTOR_FILTER=1 CARLA_COSIM_ACTOR_FILTER_CENTER_ID=AV CARLA_COSIM_ACTOR_FILTER_RADIUS=300 CARLA_COSIM_ACTOR_FILTER_HYSTERESIS=20
            export CARLA_COSIM_PHYSICS_RADIUS="${effective_radius}" CARLA_COSIM_PHYSICS_RADIUS_CENTER_ID=AV CARLA_COSIM_PHYSICS_RADIUS_HYSTERESIS=10
            export TERASIM_COSIM_STATE_FILTER=1 TERASIM_COSIM_STATE_FILTER_CENTER_ID=AV TERASIM_COSIM_STATE_FILTER_RADIUS=320 TERASIM_COSIM_STATE_MAX_VEHICLES=0
            export TERASIM_COSIM_STATE_DETAIL_RADIUS="${effective_radius}" TERASIM_COSIM_STATE_DETAIL_HYSTERESIS=10 TERASIM_COSIM_CONTINUE_ON_ACKERMANN_FEEDBACK_FAILURE=1
            export CARLA_COSIM_PROFILE_STEPS=1 CARLA_COSIM_PROFILE_WARMUP_STEPS="${PERF_WARMUP_STEPS}"
            export CARLA_COSIM_PROFILE_JSONL="${CONTAINER_OUTPUT_ROOT}/${condition}/carla_profile.jsonl"
            export TERASIM_COSIM_PROFILE_STEPS=1 TERASIM_COSIM_PROFILE_JSONL="${CONTAINER_OUTPUT_ROOT}/${condition}/terasim_profile.jsonl"
            export ENABLE_SUMO_GUI=0 SUMO_GUI_REALTIME=0 TERASIM_COSIM_CONSOLE_LOG_LEVEL=WARNING
            set +e
            docker compose -f docker-compose.ackermann-odaiba-feedback-gui.yml run --rm --name "${ACTIVE_TERASIM}" terasim_ackermann_odaiba_feedback_gui 2>&1 | tee "${condition_dir}/run.log"
            status=${PIPESTATUS[0]}
            set -e
            ACTIVE_TERASIM=""
            docker stop -t 30 "${CARLA_CONTAINER}" >/dev/null 2>&1 || true
            docker rm -f "${CARLA_CONTAINER}" >/dev/null 2>&1 || true
            if [[ "${status}" -ne 0 ]]; then echo "ERROR: ${condition} failed with status ${status}." >&2; exit "${status}"; fi
        done
    done
done
python3 scripts/summarize_ackermann_feedback_performance.py "${PERF_OUTPUT_ROOT}"
echo "Results: ${PERF_OUTPUT_ROOT}"

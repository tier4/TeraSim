#!/bin/bash
set -e

SUMO_NET_FILE=${SUMO_NET_FILE:-${NET_PATH:-examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml}}
OUTPUT_DIR=${OUTPUT_DIR:-examples/maps/odaiba_ll2}
METADATA_PATH=${METADATA_PATH:-examples/maps/odaiba_ll2/metadata.json}
SCENARIO_PATH=${SCENARIO_PATH:-examples/scenarios/cosim_odaiba_ll2_generated.yaml}
END_TIME=${END_TIME:-3600}
PERIOD=${PERIOD:-2.0}
SEED=${SEED:-2026}
AV_ROUTE_SEED=${AV_ROUTE_SEED:-$SEED}
FORCE_NEW_AV_ROUTE=${FORCE_NEW_AV_ROUTE:-0}
AV_ROUTE_FILE=${AV_ROUTE_FILE:-}
AV_ROUTE_ID=${AV_ROUTE_ID:-}

cd "$(dirname "$0")/.."

to_container_path() {
  local path="$1"
  case "$path" in
    /app/*)
      printf '%s\n' "$path"
      ;;
    "$PWD"/*)
      printf '/app/%s\n' "${path#"$PWD"/}"
      ;;
    /*)
      printf '%s\n' "$path"
      ;;
    *)
      printf '%s\n' "$path"
      ;;
  esac
}

to_container_abs_path() {
  local path
  path="$(to_container_path "$1")"
  case "$path" in
    /*)
      printf '%s\n' "$path"
      ;;
    *)
      printf '/app/%s\n' "$path"
      ;;
  esac
}

NET_PATH="$(to_container_path "$SUMO_NET_FILE")"
OUTPUT_DIR="$(to_container_path "$OUTPUT_DIR")"
METADATA_PATH="$(to_container_path "$METADATA_PATH")"
SCENARIO_PATH="$(to_container_path "$SCENARIO_PATH")"
SCENARIO_SUMO_NET_FILE=${SCENARIO_SUMO_NET_FILE:-$(to_container_abs_path "$SUMO_NET_FILE")}
SCENARIO_SUMO_CONFIG_FILE=${SCENARIO_SUMO_CONFIG_FILE:-$(to_container_abs_path "$OUTPUT_DIR")/simulation.sumocfg}
if [ -n "$AV_ROUTE_FILE" ]; then
  AV_ROUTE_FILE="$(to_container_path "$AV_ROUTE_FILE")"
fi

echo "Using SUMO net: ${SCENARIO_SUMO_NET_FILE}"
echo "Using SUMO config: ${SCENARIO_SUMO_CONFIG_FILE}"
if [ -n "$AV_ROUTE_FILE" ]; then
  echo "Using AV route file: ${AV_ROUTE_FILE}"
  if [ -n "$AV_ROUTE_ID" ]; then
    echo "Using AV route id: ${AV_ROUTE_ID}"
  fi
fi
echo "Updating scenario: ${SCENARIO_PATH}"

exec docker run --rm \
  -u "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e NET_PATH="$NET_PATH" \
  -e OUTPUT_DIR="$OUTPUT_DIR" \
  -e METADATA_PATH="$METADATA_PATH" \
  -e SCENARIO_PATH="$SCENARIO_PATH" \
  -e SCENARIO_SUMO_NET_FILE="$SCENARIO_SUMO_NET_FILE" \
  -e SCENARIO_SUMO_CONFIG_FILE="$SCENARIO_SUMO_CONFIG_FILE" \
  -e END_TIME="$END_TIME" \
  -e PERIOD="$PERIOD" \
  -e SEED="$SEED" \
  -e AV_ROUTE_SEED="$AV_ROUTE_SEED" \
  -e FORCE_NEW_AV_ROUTE="$FORCE_NEW_AV_ROUTE" \
  -e AV_ROUTE_FILE="$AV_ROUTE_FILE" \
  -e AV_ROUTE_ID="$AV_ROUTE_ID" \
  -v "$PWD:/app" \
  -w /app \
  terasim-service:cosim \
  bash -lc 'GEN_ARGS=()
  if [ "$FORCE_NEW_AV_ROUTE" = "1" ]; then
    GEN_ARGS+=(--force-new-av-route)
  fi
  if [ -n "$AV_ROUTE_FILE" ]; then
    GEN_ARGS+=(--av-route-file "$AV_ROUTE_FILE")
  fi
  if [ -n "$AV_ROUTE_ID" ]; then
    GEN_ARGS+=(--av-route-id "$AV_ROUTE_ID")
  fi
  python3 /app/scripts/generate_sumo_artifacts_from_net.py \
    --net "$NET_PATH" \
    --output-dir "$OUTPUT_DIR" \
    --metadata "$METADATA_PATH" \
    --end-time "$END_TIME" \
    --period "$PERIOD" \
    --seed "$SEED" \
    --av-route-seed "$AV_ROUTE_SEED" \
    "${GEN_ARGS[@]}" && \
  python3 /app/scripts/update_odaiba_ll2_generated_scenario.py \
    --metadata "$METADATA_PATH" \
    --scenario "$SCENARIO_PATH" \
    --sumo-net-file "$SCENARIO_SUMO_NET_FILE" \
    --sumo-config-file "$SCENARIO_SUMO_CONFIG_FILE"'

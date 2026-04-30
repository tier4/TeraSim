#!/bin/bash
set -e

NET_PATH=${NET_PATH:-examples/maps/odaiba_ll2/network.net.xml}
OUTPUT_DIR=${OUTPUT_DIR:-examples/maps/odaiba_ll2}
METADATA_PATH=${METADATA_PATH:-examples/maps/odaiba_ll2/metadata.json}
END_TIME=${END_TIME:-3600}
PERIOD=${PERIOD:-2.0}
SEED=${SEED:-2026}
AV_ROUTE_SEED=${AV_ROUTE_SEED:-$SEED}
FORCE_NEW_AV_ROUTE=${FORCE_NEW_AV_ROUTE:-0}

cd "$(dirname "$0")/.."

exec docker run --rm \
  -u "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e NET_PATH="$NET_PATH" \
  -e OUTPUT_DIR="$OUTPUT_DIR" \
  -e METADATA_PATH="$METADATA_PATH" \
  -e END_TIME="$END_TIME" \
  -e PERIOD="$PERIOD" \
  -e SEED="$SEED" \
  -e AV_ROUTE_SEED="$AV_ROUTE_SEED" \
  -e FORCE_NEW_AV_ROUTE="$FORCE_NEW_AV_ROUTE" \
  -v "$PWD:/app" \
  -w /app \
  terasim-service:cosim \
  bash -lc 'GEN_ARGS=()
  if [ "$FORCE_NEW_AV_ROUTE" = "1" ]; then
    GEN_ARGS+=(--force-new-av-route)
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
    --scenario /app/examples/scenarios/cosim_odaiba_ll2_generated.yaml'

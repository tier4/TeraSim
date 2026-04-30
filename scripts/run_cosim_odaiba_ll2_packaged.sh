#!/bin/bash
set -e

CARLA_PORT=${CARLA_PORT:-2010}
CARLA_HOST=${CARLA_HOST:-localhost}
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaibatest5}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_ll2.yaml}
TERASIM_PORT=${TERASIM_PORT:-8000}

echo "Using CARLA host: ${CARLA_HOST}"
echo "Using CARLA port: ${CARLA_PORT}"
echo "Using packaged CARLA map: ${CARLA_PACKAGE_MAP_NAME}"
echo "Using TeraSim config: ${TERASIM_CONFIG}"
echo "Using TeraSim port: ${TERASIM_PORT}"

exec docker compose -f docker-compose.cosim-odaiba-ll2.yml run --rm \
    -e CARLA_HOST="${CARLA_HOST}" \
    -e CARLA_PORT="${CARLA_PORT}" \
    -e CARLA_PACKAGE_MAP_NAME="${CARLA_PACKAGE_MAP_NAME}" \
    -e TERASIM_CONFIG="${TERASIM_CONFIG}" \
    -e TERASIM_PORT="${TERASIM_PORT}" \
    terasim_service \
    /bin/bash /app/examples/scripts/main_cosim_odaiba_ll2_packaged.sh

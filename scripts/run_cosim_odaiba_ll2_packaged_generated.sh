#!/bin/bash
set -e

if [ -n "${AV_ROUTE_FILE:-}" ]; then
  SUMO_NET_FILE=${SUMO_NET_FILE:-examples/maps/odaiba_ll2/odaiba_osmlike_network3.net.xml}
  PERIOD=${PERIOD:-2.0}
  echo "Preparing Odaiba LL2 SUMO artifacts with AV route file: ${AV_ROUTE_FILE}"
  SUMO_NET_FILE="${SUMO_NET_FILE}" \
  PERIOD="${PERIOD}" \
  AV_ROUTE_FILE="${AV_ROUTE_FILE}" \
  AV_ROUTE_ID="${AV_ROUTE_ID:-}" \
  ./scripts/prepare_odaiba_ll2_sumo_artifacts.sh
fi

CARLA_CONTAINER_NAME=${CARLA_CONTAINER_NAME:-carla-novnc-test}
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
if [ -z "${CARLA_DOCKER_NETWORK:-}" ]; then
  CARLA_DOCKER_NETWORK=$(docker inspect -f '{{range $name, $net := .NetworkSettings.Networks}}{{$name}}{{end}}' "${CARLA_CONTAINER_NAME}" 2>/dev/null || true)
fi
CARLA_DOCKER_NETWORK=${CARLA_DOCKER_NETWORK:-terasim_default}
if [ -z "${TERASIM_PORT:-}" ]; then
  TERASIM_PORT=$(python3 - <<'PY'
import socket

for port in (8001, 8002, 8010, 8100, 8000):
    sock = socket.socket()
    try:
        sock.bind(("127.0.0.1", port))
    except OSError:
        sock.close()
        continue
    sock.close()
    print(port)
    break
else:
    raise SystemExit("No free TeraSim port found")
PY
)
fi
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaiba_tl_mapping}
TERASIM_CONFIG=/app/examples/scenarios/cosim_odaiba_ll2_generated.yaml
SUMO_TO_CARLA_OFFSET_X=${SUMO_TO_CARLA_OFFSET_X:--92008.5}
SUMO_TO_CARLA_OFFSET_Y=${SUMO_TO_CARLA_OFFSET_Y:-45335.1}
SUMO_TO_CARLA_OFFSET_Z=${SUMO_TO_CARLA_OFFSET_Z:-0.0}

export CARLA_CONTAINER_NAME
export CARLA_DOCKER_NETWORK
export CARLA_HOST
export CARLA_PORT
export TERASIM_PORT
export CARLA_PACKAGE_MAP_NAME
export TERASIM_CONFIG
export SUMO_TO_CARLA_OFFSET_X
export SUMO_TO_CARLA_OFFSET_Y
export SUMO_TO_CARLA_OFFSET_Z

exec "$(dirname "$0")/run_cosim_odaiba_ll2_packaged.sh"

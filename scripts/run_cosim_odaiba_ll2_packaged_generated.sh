#!/bin/bash
set -e

CARLA_CONTAINER_NAME=${CARLA_CONTAINER_NAME:-carla-novnc-test}
CARLA_HOST_AUTODETECTED=0
if [ -z "${CARLA_HOST:-}" ]; then
  CARLA_HOST=$(docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "${CARLA_CONTAINER_NAME}" 2>/dev/null || true)
  CARLA_HOST_AUTODETECTED=1
fi
CARLA_HOST=${CARLA_HOST:-localhost}

if [ "${CARLA_HOST}" != "localhost" ] && [ "${CARLA_HOST}" != "127.0.0.1" ]; then
  if [ "${CARLA_HOST_AUTODETECTED}" = "1" ]; then
    if [ -z "${CARLA_PORT:-}" ] || [ "${CARLA_PORT:-}" = "2010" ]; then
      echo "Detected CARLA container IP ${CARLA_HOST}; using internal CARLA port 2000 instead of published port ${CARLA_PORT:-2010}"
      CARLA_PORT=2000
    fi
  else
    CARLA_PORT=${CARLA_PORT:-2000}
  fi
else
  CARLA_PORT=${CARLA_PORT:-2010}
fi
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
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaibatest5}
TERASIM_CONFIG=/app/examples/scenarios/cosim_odaiba_ll2_generated.yaml
SUMO_TO_CARLA_OFFSET_X=${SUMO_TO_CARLA_OFFSET_X:--92008.5}
SUMO_TO_CARLA_OFFSET_Y=${SUMO_TO_CARLA_OFFSET_Y:-45335.1}
SUMO_TO_CARLA_OFFSET_Z=${SUMO_TO_CARLA_OFFSET_Z:-0.0}

export CARLA_CONTAINER_NAME
export CARLA_HOST
export CARLA_PORT
export TERASIM_PORT
export CARLA_PACKAGE_MAP_NAME
export TERASIM_CONFIG
export SUMO_TO_CARLA_OFFSET_X
export SUMO_TO_CARLA_OFFSET_Y
export SUMO_TO_CARLA_OFFSET_Z

exec "$(dirname "$0")/run_cosim_odaiba_ll2_packaged.sh"

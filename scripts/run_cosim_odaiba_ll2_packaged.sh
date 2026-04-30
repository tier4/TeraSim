#!/bin/bash
set -e

CARLA_PORT=${CARLA_PORT:-2010}
CARLA_HOST=${CARLA_HOST:-localhost}
CARLA_PACKAGE_MAP_NAME=${CARLA_PACKAGE_MAP_NAME:-odaibatest5}
TERASIM_CONFIG=${TERASIM_CONFIG:-/app/examples/scenarios/cosim_odaiba_ll2.yaml}
TERASIM_PORT=${TERASIM_PORT:-8000}
ENABLE_SUMO_GUI=${ENABLE_SUMO_GUI:-1}
SUMO_DISPLAY=${SUMO_DISPLAY:-:20}
SUMO_NOVNC_PORT=${SUMO_NOVNC_PORT:-6093}
SUMO_VNC_PORT=${SUMO_VNC_PORT:-5913}
VNC_PASSWORD=${VNC_PASSWORD:-headless}
SUMO_GUI_TRACK_VEHICLE=${SUMO_GUI_TRACK_VEHICLE:-AV}
SUMO_GUI_TRACK_ZOOM=${SUMO_GUI_TRACK_ZOOM:-900}

echo "Using CARLA host: ${CARLA_HOST}"
echo "Using CARLA port: ${CARLA_PORT}"
echo "Using packaged CARLA map: ${CARLA_PACKAGE_MAP_NAME}"
echo "Using TeraSim config: ${TERASIM_CONFIG}"
echo "Using TeraSim port: ${TERASIM_PORT}"
echo "Using SUMO GUI/noVNC: ${ENABLE_SUMO_GUI}"
echo "Using SUMO noVNC port: ${SUMO_NOVNC_PORT}"
echo "Using SUMO GUI track vehicle: ${SUMO_GUI_TRACK_VEHICLE}"
echo "Using SUMO GUI track zoom: ${SUMO_GUI_TRACK_ZOOM}"

exec docker compose -f docker-compose.cosim-odaiba-ll2.yml run --rm \
    -e CARLA_HOST="${CARLA_HOST}" \
    -e CARLA_PORT="${CARLA_PORT}" \
    -e CARLA_PACKAGE_MAP_NAME="${CARLA_PACKAGE_MAP_NAME}" \
    -e TERASIM_CONFIG="${TERASIM_CONFIG}" \
    -e TERASIM_PORT="${TERASIM_PORT}" \
    -e ENABLE_SUMO_GUI="${ENABLE_SUMO_GUI}" \
    -e SUMO_DISPLAY="${SUMO_DISPLAY}" \
    -e SUMO_NOVNC_PORT="${SUMO_NOVNC_PORT}" \
    -e SUMO_VNC_PORT="${SUMO_VNC_PORT}" \
    -e VNC_PASSWORD="${VNC_PASSWORD}" \
    -e SUMO_GUI_TRACK_VEHICLE="${SUMO_GUI_TRACK_VEHICLE}" \
    -e SUMO_GUI_TRACK_ZOOM="${SUMO_GUI_TRACK_ZOOM}" \
    terasim_service \
    /bin/bash /app/examples/scripts/main_cosim_odaiba_ll2_packaged.sh

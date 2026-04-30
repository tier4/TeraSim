#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
RESOLUTION="${RESOLUTION:-1600x900x24}"
RESX="${RESX:-1600}"
RESY="${RESY:-900}"
CARLA_RPC_PORT="${CARLA_RPC_PORT:-2000}"

Xvfb "${DISPLAY}" -screen 0 "${RESOLUTION}" -ac +extension GLX +render -noreset &
sleep 0.5

openbox >/tmp/openbox.log 2>&1 &
sleep 0.5

PASS="${VNC_PASSWORD:-headless}"
x11vnc \
    -display "${DISPLAY}" \
    -rfbport 5900 \
    -passwd "${PASS}" \
    -forever \
    -shared \
    -bg \
    -o /tmp/x11vnc.log

NOVNC_WEB_DIR=""
for dir in /usr/share/novnc /opt/novnc /opt/noVNC; do
    if [[ -d "${dir}" ]]; then
        NOVNC_WEB_DIR="${dir}"
        break
    fi
done

if [[ -z "${NOVNC_WEB_DIR}" ]]; then
    echo "noVNC web directory not found" >&2
    exit 1
fi

websockify --web="${NOVNC_WEB_DIR}" 6080 localhost:5900 &
echo "noVNC: http://<HOST>:6080/vnc.html  password: ${PASS}"

cd /workspace
./CarlaUE4.sh \
    -nosound \
    -windowed \
    -ResX="${RESX}" \
    -ResY="${RESY}" \
    -carla-rpc-port="${CARLA_RPC_PORT}" \
    2>&1 | tee /tmp/carla.log

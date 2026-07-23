#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${SUMO_DISPLAY:-${DISPLAY:-:20}}"
RESOLUTION="${SUMO_RESOLUTION:-1600x900x24}"
VNC_PORT="${SUMO_VNC_PORT:-5913}"
NOVNC_PORT="${SUMO_NOVNC_PORT:-6093}"
PASS="${VNC_PASSWORD:-headless}"

XVFB_PID=""
OPENBOX_PID=""
X11VNC_PID=""
WEBSOCKIFY_PID=""

cleanup() {
    for pid in "${WEBSOCKIFY_PID}" "${X11VNC_PID}" "${OPENBOX_PID}" "${XVFB_PID}"; do
        if [[ -n "${pid}" ]]; then
            kill "${pid}" 2>/dev/null || true
            wait "${pid}" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

cleanup_stale_x_lock() {
    local display_id="${DISPLAY#*:}"
    display_id="${display_id%%.*}"

    if [[ ! "${display_id}" =~ ^[0-9]+$ ]]; then
        return
    fi

    local lock_file="/tmp/.X${display_id}-lock"
    local socket_file="/tmp/.X11-unix/X${display_id}"

    if [[ -f "${lock_file}" ]]; then
        local lock_pid
        lock_pid="$(tr -cd '0-9' < "${lock_file}" 2>/dev/null || true)"
        local lock_owner=""
        if [[ -n "${lock_pid}" && -r "/proc/${lock_pid}/comm" ]]; then
            lock_owner="$(cat "/proc/${lock_pid}/comm" 2>/dev/null || true)"
        fi

        if [[ ! "${lock_owner}" =~ ^(Xvfb|Xorg|Xwayland|X)$ ]]; then
            echo "Removing stale X lock for DISPLAY=${DISPLAY}: ${lock_file}"
            rm -f "${lock_file}" "${socket_file}"
        fi
    fi
}

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

cleanup_stale_x_lock

Xvfb "${DISPLAY}" -screen 0 "${RESOLUTION}" -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 0.5

openbox >/tmp/sumo-openbox.log 2>&1 &
OPENBOX_PID=$!
sleep 0.5

x11vnc \
    -display "${DISPLAY}" \
    -rfbport "${VNC_PORT}" \
    -passwd "${PASS}" \
    -forever \
    -shared \
    -noshm \
    -o /tmp/sumo-x11vnc.log &
X11VNC_PID=$!

websockify --web="${NOVNC_WEB_DIR}" "${NOVNC_PORT}" "localhost:${VNC_PORT}" &
WEBSOCKIFY_PID=$!

echo "SUMO noVNC: http://<HOST>:${NOVNC_PORT}/vnc.html  password: ${PASS}"

wait -n "${X11VNC_PID}" "${WEBSOCKIFY_PID}"

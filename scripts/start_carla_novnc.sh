#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
RESOLUTION="${RESOLUTION:-1600x900x24}"
RESX="${RESX:-1600}"
RESY="${RESY:-900}"
CARLA_RPC_PORT="${CARLA_RPC_PORT:-2000}"
CARLA_RENDER_OFFSCREEN="${CARLA_RENDER_OFFSCREEN:-0}"
CARLA_NULL_RHI="${CARLA_NULL_RHI:-0}"

is_enabled() {
    case "${1,,}" in
        1|true|yes|on)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

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
        if [[ -n "${lock_pid}" ]]; then
            lock_owner="$(ps -p "${lock_pid}" -o comm= 2>/dev/null || true)"
        fi

        if [[ ! "${lock_owner}" =~ ^(Xvfb|Xorg|Xwayland|X)$ ]]; then
            echo "Removing stale X lock for DISPLAY=${DISPLAY}: ${lock_file}"
            rm -f "${lock_file}" "${socket_file}"
        fi
    fi
}

if is_enabled "${CARLA_NULL_RHI}"; then
    echo "CARLA NullRHI enabled; X11 and noVNC are not started."
else
    cleanup_stale_x_lock
    Xvfb "${DISPLAY}" -screen 0 "${RESOLUTION}" -ac +extension GLX +render -noreset &
    sleep 0.5
fi

if is_enabled "${CARLA_NULL_RHI}"; then
    :
elif is_enabled "${CARLA_RENDER_OFFSCREEN}"; then
    echo "CARLA offscreen rendering enabled; Xvfb is running but no noVNC display will be started."
else
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
fi

cd /workspace
CARLA_ARGS=(
    -nosound
    -carla-rpc-port="${CARLA_RPC_PORT}"
)
if is_enabled "${CARLA_NULL_RHI}"; then
    CARLA_ARGS+=(-nullrhi)
elif is_enabled "${CARLA_RENDER_OFFSCREEN}"; then
    CARLA_ARGS+=(-RenderOffScreen)
else
    CARLA_ARGS+=(
        -windowed
        -ResX="${RESX}"
        -ResY="${RESY}"
    )
fi

./CarlaUE4.sh "${CARLA_ARGS[@]}" 2>&1 | tee /tmp/carla.log

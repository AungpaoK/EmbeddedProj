#!/usr/bin/env bash
# Start the fixed-route delivery stack, POS, restaurant map, and Pi kiosk.
set -eo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${ROBOT_TMUX_SESSION:-food-robot}"
ENV_PATH="${ENV_FILE:-${PROJECT_DIR}/.env}"

if [[ -f "$ENV_PATH" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$ENV_PATH"
    set +a
fi

quote() {
    printf '%q' "$1"
}

usage() {
    cat <<EOF
Usage: bash start_robot.sh [start|status|attach|console|stop]

  start   Start LiDAR, robot bridge, restaurant map, POS, and the Pi kiosk.
  status  Show tmux windows and check the local POS health endpoint.
  attach  Attach to the persistent tmux session.
  console Open the interactive terminal controller for missions and pickup.
  stop    Stop POS, kiosk, visualization, bridge, and LiDAR processes.

Session name: ${SESSION}
EOF
}

ACTION="${1:-start}"
if [[ $# -gt 1 ]]; then
    usage >&2
    exit 2
fi

command -v tmux >/dev/null 2>&1 || {
    echo "tmux is required. Install it with: sudo apt install tmux" >&2
    exit 1
}

case "$ACTION" in
    status)
        if ! tmux has-session -t "$SESSION" 2>/dev/null; then
            echo "Robot session '${SESSION}' is not running."
            exit 1
        fi
        tmux list-windows -t "$SESSION" -F '#{window_name}: #{pane_current_command}'
        if command -v curl >/dev/null 2>&1; then
            curl --silent --show-error --max-time 2 "http://127.0.0.1:${POS_PORT:-8765}/api/health" || true
            echo
        fi
        ;;
    attach)
        tmux has-session -t "$SESSION" 2>/dev/null || {
            echo "Robot session '${SESSION}' is not running." >&2
            exit 1
        }
        exec tmux attach-session -t "$SESSION"
        ;;
    console)
        tmux has-session -t "$SESSION" 2>/dev/null || {
            echo "Robot session '${SESSION}' is not running. Start it first with: bash $0 start" >&2
            exit 1
        }
        exec python3 "${PROJECT_DIR}/src/pos_console.py" \
            --url "http://127.0.0.1:${POS_PORT:-8765}"
        ;;
    stop)
        if ! tmux has-session -t "$SESSION" 2>/dev/null; then
            echo "Robot session '${SESSION}' is not running."
            exit 0
        fi
        # Stop the mission controller first so it sends a zero-velocity command;
        # then interrupt the bridge so its own shutdown path also stops the motors.
        for window in pos kiosk viz bridge lidar; do
            tmux send-keys -t "${SESSION}:${window}" C-c 2>/dev/null || true
            sleep 0.5
        done
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        echo "Robot session '${SESSION}' stopped."
        ;;
    start)
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac

if [[ "$ACTION" != start ]]; then
    exit 0
fi

MOTION_BACKEND="${MOTION_BACKEND:-serial}"
if [[ "${MOTION_BACKEND,,}" != "ros" ]]; then
    echo "This launcher requires MOTION_BACKEND=ros so slam_bridge is the sole owner of the motor Arduino." >&2
    echo "Set MOTION_BACKEND=ros in ${ENV_PATH} and retry." >&2
    exit 1
fi

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/ros2_ws/install/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup file not found: ${ROS_SETUP}" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -f "$ROS_WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_WS_SETUP"
fi

command -v ros2 >/dev/null 2>&1 || {
    echo "ros2 was not found after sourcing ${ROS_SETUP}." >&2
    exit 1
}
python3 -c 'import rclpy, serial, PIL, yaml' >/dev/null 2>&1 || {
    echo "Python packages rclpy, pySerial, Pillow, and PyYAML are required in the sourced ROS environment." >&2
    exit 1
}
for package in sllidar_ros2; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || {
        echo "ROS package '${package}' is not available. Build/source the ROS workspace first." >&2
        exit 1
    }
done
for map_file in maps/restaurant_map.yaml maps/restaurant_map.pgm; do
    if [[ ! -f "${PROJECT_DIR}/${map_file}" ]]; then
        echo "Restaurant map asset is missing: ${PROJECT_DIR}/${map_file}" >&2
        exit 1
    fi
done

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Robot session '${SESSION}' is already running. Use '$0 status', '$0 attach', or '$0 stop'."
    exit 0
fi

# The older standalone launcher owns the same serial devices. Require it to be
# stopped before creating the unified session to avoid duplicate device owners.
if tmux has-session -t slam 2>/dev/null; then
    echo "A legacy 'slam' session is running. Stop it first with: bash ${PROJECT_DIR}/stop_slam.sh" >&2
    exit 1
fi
if tmux has-session -t pos 2>/dev/null; then
    echo "A legacy 'pos' session is running. Stop that controller before starting this launcher." >&2
    exit 1
fi

source "${PROJECT_DIR}/src/resolve_lidar_port.sh"
LIDAR_PORT="$(resolve_lidar_port)" || exit 1

if [[ -z "${MOTION_PORT:-}" ]]; then
    shopt -s nullglob
    arduino_ports=(/dev/serial/by-id/*Arduino*)
    shopt -u nullglob
    if (( ${#arduino_ports[@]} == 1 )); then
        MOTION_PORT="${arduino_ports[0]}"
    elif (( ${#arduino_ports[@]} > 1 )); then
        echo "Multiple Arduino devices found. Set MOTION_PORT in ${ENV_PATH}." >&2
        printf '  %s\n' "${arduino_ports[@]}" >&2
        exit 1
    elif [[ -e /dev/ttyACM0 ]]; then
        MOTION_PORT=/dev/ttyACM0
    elif [[ -e /dev/ttyACM1 ]]; then
        MOTION_PORT=/dev/ttyACM1
    else
        echo "Motion Arduino not found. Set MOTION_PORT in ${ENV_PATH}." >&2
        exit 1
    fi
fi
if [[ ! -e "$MOTION_PORT" ]]; then
    echo "Motion Arduino port does not exist: ${MOTION_PORT}" >&2
    exit 1
fi

MOTION_SERIAL_BAUD="${MOTION_SERIAL_BAUD:-115200}"
LIDAR_YAW_OFFSET="${LIDAR_YAW_OFFSET:-180}"
POS_PORT="${POS_PORT:-8765}"
POS_DISPLAY="${POS_DISPLAY:-:0}"
POS_XDG_RUNTIME_DIR="${POS_XDG_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}}"
POS_XAUTHORITY="${POS_XAUTHORITY:-}"

# The XWayland authorization cookie is generated per graphical session. Read
# its current path from the live Xwayland process instead of relying on a
# hard-coded Mutter cookie suffix in .env.
ACTIVE_XWAYLAND_AUTHORITY="$(python3 - <<'PY'
import os
from pathlib import Path

for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
    try:
        raw_args = cmdline.read_bytes().split(b"\0")
    except OSError:
        continue
    args = [arg.decode(errors="replace") for arg in raw_args if arg]
    if not args or os.path.basename(args[0]) != "Xwayland":
        continue
    for index, arg in enumerate(args[:-1]):
        if arg == "-auth":
            print(args[index + 1])
            raise SystemExit(0)
PY
)"
if [[ -n "$ACTIVE_XWAYLAND_AUTHORITY" && -r "$ACTIVE_XWAYLAND_AUTHORITY" ]]; then
    # Prefer the live Mutter cookie over a generated cookie path saved from a
    # previous login. Preserve explicit, readable non-Mutter auth files.
    if [[ -z "$POS_XAUTHORITY" \
        || ! -r "$POS_XAUTHORITY" \
        || "$POS_XAUTHORITY" == */.mutter-Xwaylandauth.* ]]; then
        POS_XAUTHORITY="$ACTIVE_XWAYLAND_AUTHORITY"
    fi
fi
if [[ -z "$POS_XAUTHORITY" ]]; then
    if [[ -z "${SSH_CONNECTION:-}" && -n "${XAUTHORITY:-}" ]]; then
        POS_XAUTHORITY="$XAUTHORITY"
    fi
fi
if [[ -z "$POS_XAUTHORITY" ]]; then
    # Fall back to common X11 authorization paths if no active XWayland cookie
    # was found above.
    for candidate in \
        "${HOME}/.Xauthority" \
        "${POS_XDG_RUNTIME_DIR}"/.mutter-Xwaylandauth.* \
        "${POS_XDG_RUNTIME_DIR}/gdm/Xauthority"; do
        if [[ -r "$candidate" ]]; then
            POS_XAUTHORITY="$candidate"
            break
        fi
    done
fi

if [[ -n "${POS_BROWSER:-}" ]]; then
    command -v "$POS_BROWSER" >/dev/null 2>&1 || {
        echo "POS_BROWSER executable not found: ${POS_BROWSER}" >&2
        exit 1
    }
elif ! command -v chromium >/dev/null 2>&1 \
    && ! command -v chromium-browser >/dev/null 2>&1 \
    && ! command -v firefox >/dev/null 2>&1; then
    echo "Install Chromium or Firefox on the Pi, or set POS_BROWSER." >&2
    exit 1
fi

if ! command -v flock >/dev/null 2>&1; then
    echo "flock is required by the POS kiosk supervisor." >&2
    exit 1
fi
if command -v fuser >/dev/null 2>&1 && fuser -s -n tcp "$POS_PORT" 2>/dev/null; then
    echo "TCP port ${POS_PORT} is already in use. Stop the existing POS controller first." >&2
    exit 1
fi

ROS_PREFIX="source $(quote "$ROS_SETUP")"
if [[ -f "$ROS_WS_SETUP" ]]; then
    ROS_PREFIX+=" && source $(quote "$ROS_WS_SETUP")"
fi
PROJECT_Q="$(quote "$PROJECT_DIR")"
SRC_Q="$(quote "${PROJECT_DIR}/src")"
LIDAR_Q="$(quote "$LIDAR_PORT")"
MOTION_Q="$(quote "$MOTION_PORT")"
BAUD_Q="$(quote "$MOTION_SERIAL_BAUD")"
YAW_Q="$(quote "$LIDAR_YAW_OFFSET")"
POS_PORT_Q="$(quote "$POS_PORT")"

lidar_command="${ROS_PREFIX} && ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=${LIDAR_Q} serial_baudrate:=115200"
bridge_command="${ROS_PREFIX} && cd ${SRC_Q} && MOTION_PORT=${MOTION_Q} MOTION_SERIAL_BAUD=${BAUD_Q} INVERT_LINEAR=1 INVERT_STEER=0 INVERT_ODOM_YAW=0 ODOM_TRACK_WIDTH_FACTOR=1.185 LIDAR_OFFSET_X=0.15 LIDAR_OFFSET_Y=0.0 LIDAR_YAW_OFFSET=${YAW_Q} SELF_FILTER_RADIUS=0.195 exec python3 slam_bridge.py"
viz_command="${ROS_PREFIX} && cd ${SRC_Q} && exec python3 restaurant_visualizer.py"
pos_command="${ROS_PREFIX} && cd ${PROJECT_Q} && MOTION_BACKEND=ros LIDAR_YAW_OFFSET=${YAW_Q} POS_PORT=${POS_PORT_Q} exec bash deployPOS/run_pos_controller.sh"

kiosk_command="cd ${PROJECT_Q} && POS_DISPLAY=$(quote "$POS_DISPLAY") POS_XDG_RUNTIME_DIR=$(quote "$POS_XDG_RUNTIME_DIR") POS_XAUTHORITY=$(quote "$POS_XAUTHORITY") POS_WAYLAND_DISPLAY=$(quote "${POS_WAYLAND_DISPLAY:-}") POS_BROWSER=$(quote "${POS_BROWSER:-}") POS_URL=$(quote "http://127.0.0.1:${POS_PORT}/") exec bash deployPOS/start_pos_kiosk.sh"

run_window() {
    local name="$1"
    local command_text="$2"
    tmux new-window -d -t "$SESSION" -n "$name" "bash -lc $(quote "$command_text")"
}

tmux new-session -d -s "$SESSION" -n lidar "bash -lc $(quote "$lidar_command")"
tmux set-window-option -t "$SESSION" remain-on-exit on
run_window bridge "$bridge_command"
sleep 2
run_window viz "$viz_command"
sleep 2
run_window pos "$pos_command"
sleep 2
run_window kiosk "$kiosk_command"

if command -v xset >/dev/null 2>&1; then
    DISPLAY="$POS_DISPLAY" XAUTHORITY="$POS_XAUTHORITY" xset s off >/dev/null 2>&1 || true
    DISPLAY="$POS_DISPLAY" XAUTHORITY="$POS_XAUTHORITY" xset -dpms >/dev/null 2>&1 || true
    DISPLAY="$POS_DISPLAY" XAUTHORITY="$POS_XAUTHORITY" xset dpms force on >/dev/null 2>&1 || true
fi

echo "Robot stack started in tmux session '${SESSION}'."
echo "POS URL on the Pi: http://127.0.0.1:${POS_PORT}/"
echo "View processes:    bash ${PROJECT_DIR}/start_robot.sh attach"
echo "Check status:      bash ${PROJECT_DIR}/start_robot.sh status"
echo "Terminal control:  bash ${PROJECT_DIR}/start_robot.sh console"
echo "RViz on the PC:    ./run_rviz2_pc.sh src/scenario_view.rviz"
echo "Stop all:          bash ${PROJECT_DIR}/start_robot.sh stop"
echo "The kiosk targets DISPLAY=${POS_DISPLAY}; ensure the Pi desktop is logged in as this user."

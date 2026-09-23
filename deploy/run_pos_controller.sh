#!/usr/bin/env bash
# ROS setup scripts intentionally reference some unset variables, so avoid
# nounset while sourcing them.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_WS_SETUP="${ROS_WS_SETUP:-${HOME}/ros2_ws/install/setup.bash}"

if [[ -f "$ROS_SETUP" ]]; then
    # ROS 2 exports runtime libraries and Python modules required by main.py.
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [[ -f "$ROS_WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_WS_SETUP"
fi

exec python3 "$PROJECT_DIR/src/main.py"

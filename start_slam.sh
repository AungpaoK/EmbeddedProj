#!/usr/bin/env bash
# ==============================================================================
# start_slam.sh — สคริปต์เปิดระบบ SLAM 4 หน้าต่างอัตโนมัติด้วย tmux
# ==============================================================================
# ใช้งาน:
#   chmod +x start_slam.sh
#   ./start_slam.sh
# ==============================================================================

SESSION="slam"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${ENV_FILE:-${DIR}/.env}" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "${ENV_FILE:-${DIR}/.env}"
    set +a
fi
source "$DIR/src/resolve_lidar_port.sh"
LIDAR_PORT="$(resolve_lidar_port)" || exit 1
echo "Using RPLiDAR serial port: $LIDAR_PORT"
MOTION_PORT="${MOTION_PORT:-/dev/ttyACM0}"
MOTION_SERIAL_BAUD="${MOTION_SERIAL_BAUD:-115200}"
echo "Using Motion Arduino serial port: $MOTION_PORT"
echo "Using Motion Arduino serial baud: $MOTION_SERIAL_BAUD"

# ตรวจสอบว่ามี tmux ติดตั้งอยู่หรือไม่
if ! command -v tmux &> /dev/null; then
    echo "กำลังติดตั้ง tmux..."
    sudo apt update && sudo apt install -y tmux
fi

# ถ้ามี session เดิมค้างอยู่ ให้ปิดก่อน
tmux kill-session -t "$SESSION" 2>/dev/null

echo "กำลังสตาร์ต SLAM Session (4 หน้าต่าง)..."

# สร้าง tmux session แบบ detached
tmux new-session -d -s "$SESSION" -n "SLAM_Grid"

# แบ่งหน้าจอเป็น 4 ช่อง (2x2 Grid)
tmux split-window -h -t "$SESSION:0"
tmux split-window -v -t "$SESSION:0.0"
tmux split-window -v -t "$SESSION:0.1"

# จัด layout ให้เป็นตารางสี่เหลี่ยมสมมาตร (2x2)
tmux select-layout -t "$SESSION:0" tiled

# ------------------------------------------------------------------------------
# ช่องที่ 1 (บนซ้าย - Pane 0): RPLiDAR Node
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.0" \
  "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null && echo '=== [1] Starting RPLiDAR on $LIDAR_PORT ===' && ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=$LIDAR_PORT serial_baudrate:=115200" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 2 (บนขวา - Pane 1): SLAM Bridge (จัดการ DTR และรอ ENCODER เอง)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.1" \
  "source /opt/ros/jazzy/setup.bash && cd ~/EmbeddedProj/src && sleep 2 && echo '=== [2] Starting SLAM Bridge on $MOTION_PORT at $MOTION_SERIAL_BAUD baud ===' && MOTION_PORT=$MOTION_PORT MOTION_SERIAL_BAUD=$MOTION_SERIAL_BAUD INVERT_ODOM_YAW=0 ODOM_TRACK_WIDTH_FACTOR=1.185 LIDAR_OFFSET_X=0.15 LIDAR_OFFSET_Y=0.0 LIDAR_YAW_OFFSET=180 SELF_FILTER_RADIUS=0.195 python3 slam_bridge.py" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 3 (ล่างซ้าย - Pane 2): SLAM Toolbox (รอ 4 วินาที ให้ TF พร้อม)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.2" \
  "source /opt/ros/jazzy/setup.bash && cd ~/EmbeddedProj/src && sleep 4 && echo '=== [3] Starting SLAM Toolbox ===' && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./slam_toolbox_config.yaml" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 4 (ล่างขวา - Pane 3): Teleop Keyboard (รอ 6 วินาที แล้วพร้อมให้กดบังคับ)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.3" \
  "source /opt/ros/jazzy/setup.bash && sleep 6 && echo '=== [4] Ready for Teleop ===' && ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

# โฟกัสไปที่ช่อง Teleop Keyboard ทันที เพื่อให้พร้อมกดบังคับหุ่น
tmux select-pane -t "$SESSION:0.3"

# เปิดเข้าหน้าจอ tmux ทันที
tmux attach-session -t "$SESSION"

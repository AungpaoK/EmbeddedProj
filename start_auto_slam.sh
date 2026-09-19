#!/usr/bin/env bash
# ==============================================================================
# start_auto_slam.sh — เปิดระบบ SLAM 4 หน้าต่าง พร้อมโหมดวิ่งสำรวจอัตโนมัติ 100%
# ==============================================================================
# ใช้งาน:
#   chmod +x start_auto_slam.sh
#   ./start_auto_slam.sh
# ==============================================================================

SESSION="slam"

# ตรวจสอบว่ามี tmux ติดตั้งอยู่หรือไม่
if ! command -v tmux &> /dev/null; then
    echo "กำลังติดตั้ง tmux..."
    sudo apt update && sudo apt install -y tmux
fi

# ถ้ามี session เดิมค้างอยู่ ให้ปิดก่อน
tmux kill-session -t "$SESSION" 2>/dev/null

echo "กำลังสตาร์ต Autonomous SLAM Session (Auto Explorer Mode)..."

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
  "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash 2>/dev/null && echo '=== [1] Starting RPLiDAR ===' && ros2 launch sllidar_ros2 sllidar_a1_launch.py" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 2 (บนขวา - Pane 1): SLAM Bridge (รอ 2 วินาที)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.1" \
  "source /opt/ros/jazzy/setup.bash && cd ~/EmbeddedProj/src && sleep 2 && echo '=== [2] Starting SLAM Bridge ===' && python3 slam_bridge.py" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 3 (ล่างซ้าย - Pane 2): SLAM Toolbox (รอ 4 วินาที ให้ TF พร้อม)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.2" \
  "source /opt/ros/jazzy/setup.bash && cd ~/EmbeddedProj/src && sleep 4 && echo '=== [3] Starting SLAM Toolbox ===' && ros2 launch slam_toolbox online_async_launch.py params_file:=./slam_toolbox_config.yaml" C-m

# ------------------------------------------------------------------------------
# ช่องที่ 4 (ล่างขวา - Pane 3): Autonomous Explorer (รอ 6 วินาที แล้วเริ่มเดินสำรวจเอง)
# ------------------------------------------------------------------------------
tmux send-keys -t "$SESSION:0.3" \
  "source /opt/ros/jazzy/setup.bash && cd ~/EmbeddedProj/src && sleep 6 && echo '=== [4] Starting Auto Explorer ===' && python3 auto_explorer.py" C-m

# โฟกัสไปที่ช่อง Auto Explorer (กด Ctrl+C เพื่อหยุดได้ทุกเมื่อ)
tmux select-pane -t "$SESSION:0.3"

# เปิดเข้าหน้าจอ tmux ทันที
tmux attach-session -t "$SESSION"

#!/usr/bin/env bash
# ==============================================================================
# stop_auto_slam.sh — สั่งหยุดระบบ Auto SLAM และตัดกำลังมอเตอร์ทันที
# ==============================================================================

SESSION="slam"

echo "🛑 กำลังหยุดระบบ Auto SLAM..."

# ส่งคำสั่งหยุดฉุกเฉินผ่าน /cmd_vel เพื่อความปลอดภัย ป้องกันหุ่นยนต์วิ่งค้าง
if command -v ros2 &> /dev/null; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
fi

# ปิด tmux session
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
    echo "✓ ปิด Session [$SESSION] และหยุด Auto Explorer เรียบร้อยแล้ว"
else
    echo "ไม่มี Auto SLAM Session ทำงานอยู่"
fi

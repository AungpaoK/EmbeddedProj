#!/usr/bin/env bash
# ==============================================================================
# stop_scenario.sh — สั่งหยุดระบบ Scenario Runner และตัดกำลังขับเคลื่อนทันที
# ==============================================================================

echo "🛑 กำลังหยุดระบบ Scenario และตัดกำลังมอเตอร์..."

# 1. ส่งคำสั่งหยุดฉุกเฉินผ่าน /cmd_vel
if command -v ros2 &> /dev/null; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
fi

# 2. ปิดโปรเซส scenario_runner และ slam_bridge
pkill -f "scenario_runner.py" 2>/dev/null || true
pkill -f "slam_bridge.py" 2>/dev/null || true

echo "✓ ปิดการทำงานเรียบร้อยแล้ว"

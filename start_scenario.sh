#!/usr/bin/env bash
# ==============================================================================
# start_scenario.sh — รันระบบจำลองและนำทางหุ่นยนต์ตาม docs/scenario.md
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

# สร้างแผนที่หากยังไม่มี
if [ ! -f "maps/restaurant_map.yaml" ] || [ ! -f "maps/restaurant_map.pgm" ]; then
    echo "🗺️ กำลังสร้างไฟล์แผนที่ maps/restaurant_map..."
    python3 src/generate_scenario_map.py
fi

echo "============================================================"
echo "  🍽️ FOOD DELIVERY ROBOT — SCENARIO RUNNER (docs/scenario.md)"
echo "============================================================"
echo ""
echo "เลือกลำดับงานที่ต้องการรัน:"
echo "  1) Scenario 1: ส่งอาหารโต๊ะเดี่ยว (Serve Station -> Table 1 -> กลับครัว)"
echo "  2) Scenario 2: ส่งอาหาร 2 โต๊ะพร้อมกัน (Table 1 -> Table 2 -> กลับครัว)"
read -p "เลือกข้อ (1 หรือ 2) [ค่าเริ่มต้น: 1]: " SCENARIO_CHOICE
SCENARIO_CHOICE="${SCENARIO_CHOICE:-1}"

echo ""
echo "เลือกโหมดการทำงาน:"
echo "  1) Simulation Mode (จำลองการวิ่ง 2D และสแกน LiDAR ใน RViz2 ไม่ต้องใช้หุ่นจริง)"
echo "  2) Real Robot Mode (เชื่อมต่อสั่งวิ่งบนตัวหุ่นยนต์จริง)"
read -p "เลือกโหมด (1 หรือ 2) [ค่าเริ่มต้น: 1]: " MODE_CHOICE
MODE_CHOICE="${MODE_CHOICE:-1}"

MODE_ARG="--sim"
if [ "$MODE_CHOICE" == "2" ]; then
    MODE_ARG="--robot"
fi

echo ""
echo "เลือกระยะทางในการวิ่งทดสอบ (เลือกตามขนาดพื้นที่จริงของห้อง):"
echo "  1) ระยะมาตรฐานตามผังร้าน (Junction: 2.0m, Table: 0.6m — ต้องใช้พื้นที่ยาวอย่างน้อย 3m)"
echo "  2) ระยะย่อสำหรับห้องทดสอบขนาดเล็ก (Junction: 1.0m, Table 0.4m — ใช้พื้นที่ยาวเพียง 1.5m)"
read -p "เลือกระยะ (1 หรือ 2) [ค่าเริ่มต้น: 1]: " SCALE_CHOICE
SCALE_CHOICE="${SCALE_CHOICE:-1}"

if [ "$SCALE_CHOICE" == "2" ]; then
    export JUNCTION_X="1.0"
    export TABLE1_Y="0.4"
    export TABLE2_Y="0.4"
    echo "  📏 ใช้ระยะย่อสำหรับห้องทดสอบ: Junction = 1.0m, Table = 0.4m"
else
    export JUNCTION_X="2.0"
    export TABLE1_Y="0.6"
    export TABLE2_Y="0.6"
    echo "  📏 ใช้ระยะมาตรฐาน: Junction = 2.0m, Table = 0.6m"
fi

echo ""
echo "🚀 กำลังเริ่มต้นระบบนำทาง..."
echo "  - Scenario : $SCENARIO_CHOICE"
echo "  - Mode     : $MODE_ARG"
echo "  - Distance : Junction=${JUNCTION_X}m, Table=${TABLE1_Y}m"
echo ""
echo "💡 หากต้องการดูภาพแผนที่ 3D และตัวหุ่นยนต์ใน RViz2 ให้เปิดอีก Terminal หนึ่งแล้วรัน:"
echo "     ./run_rviz2_pc.sh src/scenario_view.rviz"
echo "============================================================"
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

BRIDGE_PID=""
MOTION_PORT="${MOTION_PORT:-/dev/ttyACM0}"
BRIDGE_LOG="/tmp/scenario_slam_bridge.log"

cleanup() {
    echo ""
    echo "🛑 กำลังตัดกำลังขับเคลื่อนและหยุดหุ่นยนต์..."
    ros2 topic pub --once -w 0 /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
    if [ -n "$BRIDGE_PID" ]; then
        echo "🔌 กำลังปิด Hardware Bridge (PID: $BRIDGE_PID)..."
        kill -SIGINT "$BRIDGE_PID" 2>/dev/null || kill -9 "$BRIDGE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# หากเลือก Real Robot Mode
if [ "$MODE_CHOICE" == "2" ]; then
    # ตรวจสอบว่าอยู่บน Raspberry Pi ที่ต่อสาย Serial กับ Arduino หรือไม่
    if [ -e "/dev/ttyACM0" ] || [ -e "/dev/ttyACM1" ]; then
        if ! pgrep -f "slam_bridge.py" > /dev/null; then
            echo "🔌 กำลังเปิดใช้งาน SLAM Hardware Bridge เพื่อเชื่อมต่อไปยัง Arduino..."
            : > "$BRIDGE_LOG"
            MOTION_PORT="$MOTION_PORT" \
            INVERT_ODOM_YAW=0 \
            ODOM_TRACK_WIDTH_FACTOR=1.185 \
            LIDAR_OFFSET_X=0.15 \
            LIDAR_OFFSET_Y=0.0 \
            SELF_FILTER_RADIUS=0.22 \
                python3 src/slam_bridge.py > "$BRIDGE_LOG" 2>&1 &
            BRIDGE_PID=$!
            # ตรวจว่า bridge เปิด serial กับ Arduino ได้จริง ไม่ใช่แค่โปรเซสยังอยู่
            for _ in {1..30}; do
                if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
                    break
                fi
                if grep -q "Opened Arduino Motion Port on " "$BRIDGE_LOG"; then
                    break
                fi
                sleep 0.2
            done
            if ! grep -q "Opened Arduino Motion Port on " "$BRIDGE_LOG"; then
                echo "❌ เปิด Arduino serial ไม่สำเร็จผ่าน $MOTION_PORT"
                echo "บันทึกข้อผิดพลาดจาก bridge:"
                cat "$BRIDGE_LOG"
                exit 1
            fi
            echo "✓ Bridge เปิดพอร์ต Arduino สำเร็จ (PID: $BRIDGE_PID)"
        else
            echo "✓ ใช้ slam_bridge.py ที่กำลังทำงานอยู่แล้ว"
        fi
    else
        echo "🌐 Real Robot Mode (เชื่อมต่อผ่าน ROS 2 Network):"
        echo "   กรุณาตรวจสอบว่าบนบอร์ด Raspberry Pi มี slam_bridge.py กำลังทำงานอยู่"
    fi
fi

# หากรันบนเครื่อง PC ที่ไม่มี native rclpy ให้รันผ่าน Docker container โดยอัตโนมัติ
if ! python3 -c "import rclpy" 2>/dev/null; then
    if command -v docker &>/dev/null && docker image inspect osrf/ros:jazzy-desktop &>/dev/null; then
        echo "🐳 ตรวจพบว่ารันบน PC Host (กำลังเปิดใช้งานผ่าน Docker osrf/ros:jazzy-desktop)..."
        docker run -it --rm --net=host --ipc=host \
            -e JUNCTION_X="$JUNCTION_X" -e TABLE1_Y="$TABLE1_Y" -e TABLE2_Y="$TABLE2_Y" \
            -v "$DIR:/workspace" osrf/ros:jazzy-desktop \
            bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace && python3 src/scenario_runner.py --scenario '$SCENARIO_CHOICE' $MODE_ARG"
        exit 0
    fi
fi

python3 src/scenario_runner.py --scenario "$SCENARIO_CHOICE" $MODE_ARG

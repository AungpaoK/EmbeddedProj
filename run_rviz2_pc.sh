#!/usr/bin/env bash
# ==============================================================================
# run_rviz2_pc.sh — รัน RViz2 บนเครื่อง PC ผ่าน Docker เพื่อดึงภาพ Map/Scan จาก Raspberry Pi
# ==============================================================================
# ทำไมต้องรันบน PC:
#   1. ไม่กิน CPU/RAM ของ Raspberry Pi 4 (SLAM + LiDAR ต้องการทรัพยากรสูง)
#   2. เรนเดอร์ 3D ลื่นไหล 60 FPS ด้วยการ์ดจอ PC
#   3. แค่อยู่ใน WiFi เดียวกัน ROS 2 DDS จะค้นหาเจออัตโนมัติ
# ==============================================================================

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ_CONFIG="${DIR}/src/slam_view.rviz"

echo "=== [Garvis] เตรียมรัน RViz2 บนคอมพิวเตอร์ ==="

# 1. อนุญาตสิทธิ์ X11 เพื่อให้ Docker เรนเดอร์หน้าต่าง GUI ออกจอได้
echo "1. อนุญาตสิทธิ์การแสดงผล X11..."
xhost +local:root > /dev/null 2>&1 || true

# 2. ตรวจสอบว่ามี Docker พร้อมหรือไม่
if ! command -v docker &> /dev/null; then
    echo "ERROR: ไม่พบ docker ในเครื่อง กรุณาติดตั้ง docker ก่อน"
    exit 1
fi

echo "2. กำลังสตาร์ต RViz2 ใน Docker (อิง ROS 2 Jazzy)..."
echo "   (หากรันครั้งแรก ระบบจะดาวน์โหลด image อัตโนมัติ)"

# 3. รัน RViz2 ด้วย host network และ share display
docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY="${DISPLAY}" \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${DIR}:/workspace" \
  osrf/ros:jazzy-desktop \
  bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d /workspace/src/slam_view.rviz"

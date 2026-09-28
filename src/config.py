#!/usr/bin/env python3
"""
config.py — Robot Physical Constants & Waypoint Map
=====================================================
ค่าคงที่ทางกายภาพของหุ่นยนต์และตำแหน่ง Waypoint ของร้านอาหาร

ปรับค่า JUNCTION_X, TABLE1_Y, TABLE2_Y ให้ตรงกับสนามจริงก่อนรัน
"""

import os

from env_loader import load_dotenv


# Load project-level configuration before reading any environment-backed values.
load_dotenv()

# ===========================================================
# Robot Physical Parameters (ซิงค์กับ robotconfig.h)
# ===========================================================
WHEEL_RADIUS: float = 0.065       # m  (6.5 cm)
WHEEL_BASE: float = 0.343         # m  (34.3 cm)
TICKS_PER_REV: float = 1920.0    # ticks/revolution
METERS_PER_TICK: float = (2.0 * 3.14159265 * WHEEL_RADIUS) / TICKS_PER_REV

# Encoder calibration factors.  Keep these at 1.0 until the calibration test
# has produced measured values for the actual floor, wheels, and payload.
LEFT_TICK_SCALE: float = float(os.environ.get("LEFT_TICK_SCALE", "1.0"))
RIGHT_TICK_SCALE: float = float(os.environ.get("RIGHT_TICK_SCALE", "1.0"))

# The ROS odometry track width is intentionally configurable because skid
# steering changes with floor friction and payload.  The historical default is
# retained as a compatibility fallback, but deployments should override it
# after running odometry_calibration_test.py on the real floor.
ODOM_TRACK_WIDTH_FACTOR: float = float(
    os.environ.get("ODOM_TRACK_WIDTH_FACTOR", os.environ.get("SKID_FACTOR", "1.185"))
)

# ===========================================================
# Serial Ports (Raspberry Pi device mapping)
# LiDAR uses a USB serial adapter; Motion Arduino uses the Uno CDC serial port.
# ===========================================================
MOTION_SERIAL_PORT: str = "/dev/ttyACM0"  # Arduino #1 (Motion)
SHELF_SERIAL_PORT: str = "none"            # Arduino #2 (Shelf) — ตั้งเป็น "none" เมื่อยังไม่ได้ต่อ (ใช้ VirtualShelf แทน)
SERIAL_BAUD: int = 115200
# Keep motion baud separate from shelf baud.  Arduino_1_Motion.ino uses
# Serial.begin(115200), so the fallback must match even when tmux does not
# inherit a project-specific environment variable.
MOTION_SERIAL_BAUD: int = int(os.environ.get("MOTION_SERIAL_BAUD", "115200"))
SERIAL_TIMEOUT: float = 1.0

# ===========================================================
# Waypoint Coordinates (หน่วย: เมตร)
#   กำหนดให้ Serve Station = (0, 0) หันหน้าไปทาง +X
#   Junction = (JUNCTION_X, 0)
#   Table 1 จุดจอดหน้าโต๊ะอยู่ทางซ้าย (Y บวก) = (JUNCTION_X, +TABLE1_Y)
#   Table 2 จุดจอดหน้าโต๊ะอยู่ทางขวา (Y ลบ)  = (JUNCTION_X, -TABLE2_Y)
# ===========================================================
JUNCTION_X: float = float(os.environ.get("JUNCTION_X", "2.0"))      # m — ระยะทางตรงจากครัวถึงทางแยก
TABLE1_Y: float = float(os.environ.get("TABLE1_Y", "0.6"))        # m — ระยะจากทางแยกถึงจุดจอดหน้าโต๊ะ 1
TABLE2_Y: float = float(os.environ.get("TABLE2_Y", "0.6"))        # m — ระยะจากทางแยกถึงจุดจอดหน้าโต๊ะ 2

# Raspberry Pi address used by the PC-side ROS 2/RViz helper.
# Accept the requested lowercase spelling as well as the conventional uppercase
# spelling and the old PI_IP variable.
IP_ADDRESS: str = (
    os.environ.get("IP_ADDRESS")
    or os.environ.get("ip_address")
    or os.environ.get("PI_IP")
    or "172.30.81.226"
)

WAYPOINTS: dict = {
    "home":      (0.0,        0.0),
    "junction":  (JUNCTION_X, 0.0),
    "table_1":   (JUNCTION_X, +TABLE1_Y),
    "table_2":   (JUNCTION_X, -TABLE2_Y),
}

# ===========================================================
# Motion Tolerances & Timing
# ===========================================================
ARRIVAL_TOLERANCE_M: float = 0.05      # m  — ระยะยอมรับว่า "ถึงแล้ว"
# Table legs use a slightly wider tolerance because the final pose is a
# deliberate stand-off point in front of a physical table. This does not
# disable LiDAR safety before the robot reaches this odometry distance.
TABLE_STOP_TOLERANCE_M: float = float(
    os.environ.get("TABLE_STOP_TOLERANCE_M", "0.10")
)  # m — tolerance ของจุดจอดหน้าโต๊ะ
HEADING_TOLERANCE_DEG: float = 5.0     # °  — มุมยอมรับว่าตรงทิศทางแล้ว
WAYPOINT_POSITION_TOLERANCE_M: float = float(
    os.environ.get("WAYPOINT_POSITION_TOLERANCE_M", "0.10")
)
WAYPOINT_HEADING_TOLERANCE_DEG: float = float(
    os.environ.get("WAYPOINT_HEADING_TOLERANCE_DEG", "5.0")
)

# Fixed-route cross-track controller.  The feature is enabled by default so
# the navigation fix is active for a normal run; set CROSS_TRACK_CONTROL=0 for
# an A/B comparison with the legacy heading-only controller.
CROSS_TRACK_CONTROL_ENABLED: bool = os.environ.get(
    "CROSS_TRACK_CONTROL", "1"
).strip().lower() in {"1", "true", "yes", "on"}
CROSS_TRACK_GAIN: float = float(os.environ.get("CROSS_TRACK_GAIN", "1.2"))
HEADING_GAIN: float = float(os.environ.get("HEADING_GAIN", "1.8"))
MAX_CROSS_TRACK_CORRECTION: float = float(
    os.environ.get("MAX_CROSS_TRACK_CORRECTION", "0.30")
)
MIN_LINEAR_SPEED: float = float(os.environ.get("MIN_LINEAR_SPEED", "0.10"))
TELEMETRY_INTERVAL_S: float = float(
    os.environ.get("NAV_TELEMETRY_INTERVAL_S", "0.50")
)
MOTION_COMMAND_TIMEOUT_S: float = 30.0 # s  — timeout รอ STATUS:DONE จาก Arduino
PICKUP_WAIT_TIMEOUT_S: float = 120.0   # s  — timeout รอลูกค้าหยิบอาหาร (Manual Override จะข้ามได้)

# ===========================================================
# Shelf Configuration
# ===========================================================
NUM_SHELVES: int = 2   # จำนวนชั้นวางอาหาร (ชั้น 1 = Floor 2, ชั้น 2 = Floor 3)

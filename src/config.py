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

# ===========================================================
# Serial Ports (Raspberry Pi device mapping)
# LiDAR uses a USB serial adapter; Motion Arduino uses the Uno CDC serial port.
# ===========================================================
MOTION_SERIAL_PORT: str = "/dev/ttyACM0"  # Arduino #1 (Motion)
SHELF_SERIAL_PORT: str = "none"            # Arduino #2 (Shelf) — ตั้งเป็น "none" เมื่อยังไม่ได้ต่อ (ใช้ VirtualShelf แทน)
SERIAL_BAUD: int = 115200
# Keep motion baud separate from shelf baud.  The current Uno firmware is
# observed on the Pi at 9600 baud; make this configurable so a replacement
# board/firmware can be switched without editing the bridge.
MOTION_SERIAL_BAUD: int = int(os.environ.get("MOTION_SERIAL_BAUD", "9600"))
SERIAL_TIMEOUT: float = 1.0

# ===========================================================
# Waypoint Coordinates (หน่วย: เมตร)
#   กำหนดให้ Serve Station = (0, 0) หันหน้าไปทาง +X
#   Junction = (JUNCTION_X, 0)
#   Table 1 อยู่ทางซ้าย (Y บวก) = (JUNCTION_X, +TABLE1_Y)
#   Table 2 อยู่ทางขวา (Y ลบ)  = (JUNCTION_X, -TABLE2_Y)
# ===========================================================
JUNCTION_X: float = float(os.environ.get("JUNCTION_X", "2.0"))      # m — ระยะทางตรงจากครัวถึงทางแยก
TABLE1_Y: float = float(os.environ.get("TABLE1_Y", "0.6"))        # m — ระยะทางจากทางแยกถึงโต๊ะ 1 (ซ้าย/เหนือ)
TABLE2_Y: float = float(os.environ.get("TABLE2_Y", "0.6"))        # m — ระยะทางจากทางแยกถึงโต๊ะ 2 (ขวา/ใต้)

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
HEADING_TOLERANCE_DEG: float = 5.0     # °  — มุมยอมรับว่าตรงทิศทางแล้ว
MOTION_COMMAND_TIMEOUT_S: float = 30.0 # s  — timeout รอ STATUS:DONE จาก Arduino
PICKUP_WAIT_TIMEOUT_S: float = 120.0   # s  — timeout รอลูกค้าหยิบอาหาร (Manual Override จะข้ามได้)

# ===========================================================
# Shelf Configuration
# ===========================================================
NUM_SHELVES: int = 2   # จำนวนชั้นวางอาหาร (ชั้น 1 = Floor 2, ชั้น 2 = Floor 3)

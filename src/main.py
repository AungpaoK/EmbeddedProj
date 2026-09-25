#!/usr/bin/env python3
"""
main.py — Food Delivery Robot Entry Point (Unified ROS 2 Hybrid System)
=======================================================================
จุดเริ่มต้นโปรแกรม: เชื่อมต่อ Serial (Dual Arduino), เริ่มต้นระบบ ROS 2 (LiDAR / SLAM)
เปิดใช้งาน LiDAR Safety Guard และ Continuous Waypoint Controller แล้วรัน Main FSM

การรัน:
    python3 main.py

ตัวเลือก Environment Variable:
    MOTION_BACKEND     — serial (เดิม) หรือ ros (ให้ slam_bridge ถือ Arduino)
    MOTION_PORT        — Serial port ของ Arduino #1  (default: /dev/ttyACM0)
    SHELF_PORT         — Serial port ของ Arduino #2  (default: none)
    BAUD_RATE          — Baud rate ทั้งสอง port       (default: 115200)
    LIDAR_YAW_OFFSET   — องศาชดเชยการวาง LiDAR เทียบกับหน้ารถ (default: 0.0)
    LIDAR_STOP_DIST    — ระยะหยุดฉุกเฉิน LiDAR (เมตร, default: 0.30)
    LOG_LEVEL          — DEBUG / INFO / WARNING        (default: INFO)
"""

import logging
import os
import sys
import math
import signal
import threading
import time
try:
    import serial
except ImportError:
    serial = None

# Local imports
from config import (
    MOTION_SERIAL_PORT,
    SHELF_SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
)
from odometry import Odometry, RosOdometry
from motion_client import MotionClient, RosMotionClient
from shelf_client import ShelfClient, VirtualShelfClient
from delivery_fsm import DeliveryFSM
from pos_server import PosBridge, PosServer
from lidar_safety import LidarSafetyGuard
from waypoint_controller import WaypointController

# Optional ROS 2 Integration
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry as RosOdomMsg
    from std_msgs.msg import Bool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


# ===========================================================
# Logging Setup
# ===========================================================
def _setup_logging() -> None:
    level_str = os.environ.get("LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_str, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )


# ===========================================================
# Serial Connection Helper & Auto-detection
# ===========================================================
def _find_motion_port(preferred: str) -> str:
    """ค้นหาพอร์ต Arduino #1 อัตโนมัติ หาก preferred port ไม่มีอยู่จริง"""
    if os.path.exists(preferred):
        return preferred
    # Arduino ใช้ ttyACM; อย่าหยิบ /dev/ttyUSB1 ซึ่งเป็น LiDAR มาเปิดเป็นมอเตอร์
    for candidate in ["/dev/ttyACM0", "/dev/ttyACM1"]:
        if os.path.exists(candidate):
            logging.getLogger(__name__).info(f"[Motion] Auto-detected port: {candidate}")
            return candidate
    return preferred


def _open_serial(port: str, baud: int, timeout: float, label: str, optional: bool = False):
    if not port or port.lower() in ("none", "null", "false", "mock", ""):
        logging.getLogger(__name__).info(f"[{label}] Port disabled ('{port}').")
        return None
    if serial is None:
        if optional:
            logging.getLogger(__name__).warning(f"[{label}] pyserial is not installed. Running in Optional/Virtual mode.")
            return None
        logging.getLogger(__name__).critical(f"[{label}] pyserial is not installed! Run: sudo apt install python3-serial")
        sys.exit(1)
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        logging.getLogger(__name__).info(f"[{label}] Connected: {port} @ {baud} baud")
        return ser
    except (serial.SerialException, FileNotFoundError, OSError) as e:
        if optional:
            logging.getLogger(__name__).warning(f"[{label}] Port {port} not available: {e}. (Running in Optional/Virtual mode)")
            return None
        logging.getLogger(__name__).critical(f"[{label}] Cannot open {port}: {e}")
        sys.exit(1)


# ===========================================================
# ROS 2 Bridge Node (Background Subscriptions)
# ===========================================================
if HAS_ROS2:
    class DeliveryRobotRosNode(Node):
        def __init__(self, safety_guard: LidarSafetyGuard):
            super().__init__("delivery_robot_node")
            self._safety = safety_guard
            self.latest_odom_pose = None
            self.latest_odom_time = 0.0
            self.motion_ready = False
            self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

            # 1. Subscribe /scan จาก sllidar_ros2
            self._scan_sub = self.create_subscription(
                LaserScan,
                "/scan",
                self._safety.ros2_scan_callback,
                10,
            )

            # 2. Fixed-route navigation uses encoder odometry. The restaurant
            # map is a visualization, not an AMCL/Nav2 localization source.
            self._odom_sub = self.create_subscription(
                RosOdomMsg,
                "/odom",
                self._odom_callback,
                10,
            )
            self._ready_sub = self.create_subscription(
                Bool,
                "/arduino/ready",
                self._ready_callback,
                10,
            )
            self.get_logger().info("ROS 2 DeliveryRobotRosNode initialized.")

        def _odom_callback(self, msg: RosOdomMsg):
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.latest_odom_pose = (pos.x, pos.y, math.atan2(siny_cosp, cosy_cosp))
            self.latest_odom_time = time.monotonic()

        def _ready_callback(self, msg: Bool):
            self.motion_ready = bool(msg.data)

        def odom_is_fresh(self, max_age_s: float = 1.0) -> bool:
            return bool(
                self.latest_odom_pose is not None
                and self.latest_odom_time > 0.0
                and time.monotonic() - self.latest_odom_time <= max_age_s
            )

        def publish_cmd_vel(
            self,
            linear_v: float,
            angular_w: float,
            *,
            allow_when_not_ready: bool = False,
        ) -> bool:
            if not self.motion_ready and not allow_when_not_ready:
                return False
            msg = Twist()
            msg.linear.x = float(linear_v)
            msg.angular.z = float(angular_w)
            self._cmd_vel_pub.publish(msg)
            return True


# ===========================================================
# Graceful Shutdown Handler
# ===========================================================
def _make_shutdown_handler(odom: Odometry, shelf: ShelfClient, motion: MotionClient):
    def _handler(sig, frame):
        print("\n[main] Shutting down... stopping motors")
        motion.stop_continuous()
        motion.stop()
        odom.stop()
        shelf.stop()
        if HAS_ROS2 and rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    return _handler


# ===========================================================
# Main
# ===========================================================
def main() -> None:
    _setup_logging()
    logger = logging.getLogger(__name__)

    # --- Read runtime backend and port overrides ---
    motion_backend = os.environ.get("MOTION_BACKEND", "serial").strip().lower()
    if motion_backend not in {"serial", "ros"}:
        logger.error("Unsupported MOTION_BACKEND=%r; use 'serial' or 'ros'.", motion_backend)
        sys.exit(2)

    motion_port = os.environ.get("MOTION_PORT", MOTION_SERIAL_PORT)
    if motion_backend == "serial":
        motion_port = _find_motion_port(motion_port)
    shelf_port  = os.environ.get("SHELF_PORT",  SHELF_SERIAL_PORT)
    baud        = int(os.environ.get("BAUD_RATE", SERIAL_BAUD))
    yaw_offset  = float(os.environ.get("LIDAR_YAW_OFFSET", "0.0"))
    stop_dist   = float(os.environ.get("LIDAR_STOP_DIST", "0.30"))

    logger.info("=" * 55)
    logger.info("  Food Delivery Robot — Unified ROS 2 Hybrid")
    logger.info("=" * 55)
    logger.info(
        "  Motion backend   : %s",
        "ROS topics (/cmd_vel, /odom)" if motion_backend == "ros" else f"serial ({motion_port})",
    )
    logger.info(f"  Shelf Arduino    : {shelf_port} (Optional)")
    logger.info(f"  LiDAR Yaw Offset : {yaw_offset}°")
    logger.info(f"  LiDAR Stop Dist  : {stop_dist} m")
    logger.info(f"  ROS 2 Status     : {'Available' if HAS_ROS2 else 'Standalone / No ROS 2'}")

    # --- LiDAR Safety Guard ---
    safety_guard = LidarSafetyGuard(
        stop_distance_m=stop_dist,
        front_cone_deg=35.0,
        min_clearance_m=0.22,
        yaw_offset_deg=yaw_offset,
    )

    # --- Start the selected motion transport ---
    ros_node = None
    ros_thread = None
    if motion_backend == "ros":
        if not HAS_ROS2:
            logger.critical("MOTION_BACKEND=ros requires ROS 2 (rclpy and message packages).")
            sys.exit(1)
        rclpy.init()
        ros_node = DeliveryRobotRosNode(safety_guard)
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        ros_thread.start()
        motion_ser = None
        motion = RosMotionClient(ros_node)
        odometry = RosOdometry(ros_node)
    else:
        motion_ser = _open_serial(motion_port, baud, SERIAL_TIMEOUT, "Motion")
        motion = MotionClient(motion_ser)
        odometry = Odometry(motion_ser)

    # Shelf serial remains owned by main.py; it is a separate optional device.
    shelf_ser = _open_serial(shelf_port, baud, SERIAL_TIMEOUT, "Shelf", optional=True)

    # Shelf Subsystem (Hardware or Virtual Fallback)
    if shelf_ser is not None:
        shelf = ShelfClient(shelf_ser)
    else:
        logger.info("[main] Arduino #2 (Shelf) not connected. Running with VirtualShelfClient.")
        shelf = VirtualShelfClient(auto_dispatch=False)

    # --- ROS 2 Node Spin (Optional for serial backend) ---
    if HAS_ROS2 and ros_node is None:
        rclpy.init()
        ros_node = DeliveryRobotRosNode(safety_guard)
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        ros_thread.start()
        logger.info("[main] ROS 2 background subscriber thread started.")

    # --- Pose Provider (mission-local wheel odometry) ---
    def get_pose():
        return odometry.pose

    # --- Closed-Loop Waypoint Controller ---
    waypoint_ctrl = WaypointController(
        motion=motion,
        safety_guard=safety_guard,
        pose_provider=get_pose,
        ready_provider=(
            (lambda: ros_node.motion_ready)
            if motion_backend == "ros"
            else (lambda: True)
        ),
        pose_fresh_provider=(
            (lambda: ros_node.odom_is_fresh())
            if motion_backend == "ros"
            else (lambda: True)
        ),
    )

    # --- Register Ctrl+C Shutdown ---
    signal.signal(signal.SIGINT, _make_shutdown_handler(odometry, shelf, motion))

    # --- Start Background Threads ---
    odometry.start()
    shelf.start()

    logger.info("[main] All subsystems started. Launching Main FSM.")

    # --- Local POS Server and Main FSM ---
    pos_bridge = PosBridge()
    fsm = DeliveryFSM(
        motion=motion,
        odometry=odometry,
        shelf=shelf,
        pos_bridge=pos_bridge,
        waypoint_controller=waypoint_ctrl,
    )
    pos_server = None
    try:
        pos_port = int(os.environ.get("POS_PORT", "8765"))
        pos_server = PosServer(pos_bridge, host="127.0.0.1", port=pos_port)
        pos_server.start()
        logger.info("[main] POS touchscreen available at %s", pos_server.url)
        fsm.run()
    except Exception as e:
        logger.exception(f"[main] Unhandled FSM exception: {e}")
        motion.stop_continuous()
        motion.stop()
        raise
    finally:
        logger.info("[main] Cleaning up...")
        if pos_server is not None:
            pos_server.stop()
        motion.stop_continuous()
        odometry.stop()
        shelf.stop()
        if HAS_ROS2 and rclpy.ok():
            rclpy.shutdown()
        if motion_ser is not None:
            motion_ser.close()
        if shelf_ser is not None:
            shelf_ser.close()
        logger.info("[main] Shutdown complete.")


if __name__ == "__main__":
    main()

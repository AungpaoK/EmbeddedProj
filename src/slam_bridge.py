#!/usr/bin/env python3
"""
slam_bridge.py — Standalone SLAM & Teleop Bridge for ROS 2
===========================================================
เครื่องมือสำหรับทดสอบ SLAM โดยเฉพาะ (ตัดเรื่องระบบส่งอาหารออกไปก่อน)

หน้าที่:
  1. บริดจ์คำสั่งความเร็ว /cmd_vel (Twist) จากคีย์บอร์ด -> ส่ง V:v_L,v_R ให้ Arduino #1
  2. อ่าน Encoder จาก Arduino #1 -> Publish /odom และบรอดคาสต์ TF: odom -> base_link
  3. บรอดคาสต์ Static TF: base_link -> laser_frame (พร้อมชดเชย Yaw Offset)
  4. หากยังไม่ได้ต่อ Arduino #1 จะเปิด Dummy TF ให้ทดสอบสแกนด้วย LiDAR เพียวๆ ได้
"""

import os
import sys
import math
import time
import threading
import logging

try:
    import serial
except ImportError:
    serial = None

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry as OdomMsg
    import tf2_ros
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from config import (
    WHEEL_BASE,
    WHEEL_RADIUS,
    TICKS_PER_REV,
    METERS_PER_TICK,
    MOTION_SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SLAMBridge")


class SlamBridgeNode(Node):
    def __init__(self, ser=None, yaw_offset_deg: float = 0.0):
        super().__init__("slam_bridge_node")
        self._ser = ser
        self._yaw_offset = math.radians(yaw_offset_deg)

        # Odometry State
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._prev_l = 0
        self._prev_r = 0
        self._first_enc = True

        # TF Broadcasters
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publishers & Subscribers
        self._odom_pub = self.create_publisher(OdomMsg, "/odom", 10)
        self._cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)

        # Direction inversion settings (แก้ปัญหามอเตอร์กลับขั้ว / เดินถอยหลัง / เลี้ยวกลับด้าน)
        self._invert_linear = os.environ.get("INVERT_LINEAR", "1") == "1"
        self._invert_angular = os.environ.get("INVERT_ANGULAR", "0") == "1"
        self._invert_left_enc = os.environ.get("INVERT_LEFT_ENC", "0") == "1"
        self._invert_right_enc = os.environ.get("INVERT_RIGHT_ENC", "1") == "1"
        logger.info(
            f"Drive Direction: InvertLinear={self._invert_linear}, InvertAngular={self._invert_angular}, "
            f"InvertLeftEnc={self._invert_left_enc}, InvertRightEnc={self._invert_right_enc}"
        )

        # Broadcast Static TF: base_link -> laser_frame (ทิศทางของ LiDAR)
        self._broadcast_static_laser_tf()

        # Timer สำหรับ Publish Odometry และ TF ที่ 20 Hz
        self._timer = self.create_timer(0.05, self._publish_odom_and_tf)

        # Serial Reader Thread (ถ้าต่อ Arduino)
        self._running = True
        if self._ser:
            self._reader_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
            self._reader_thread.start()
            logger.info("Connected to Arduino #1 Motion Controller.")
        else:
            logger.warning("No Arduino connected. Running with Pure Laser / Static Odom mode.")

    def _broadcast_static_laser_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"

        # ติดตั้งหน้ารถเยื้อง 15cm
        t.transform.translation.x = 0.15
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.10

        half_yaw = self._yaw_offset / 2.0
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)

        self._static_tf_broadcaster.sendTransform(t)

    def _cmd_vel_callback(self, msg: Twist):
        """รับความเร็วจาก teleop แล้วส่ง V:left,right ให้ Arduino"""
        if not self._ser:
            return

        # สลับทิศทางหากตั้งค่า Invert ไว้ (เช่น กด i แล้วถอยหลัง)
        v = -msg.linear.x if self._invert_linear else msg.linear.x
        w = -msg.angular.z if self._invert_angular else msg.angular.z

        v_l = v - (w * WHEEL_BASE / 2.0)
        v_r = v + (w * WHEEL_BASE / 2.0)

        cmd = f"V:{v_l:.3f},{v_r:.3f}\n"
        try:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
        except Exception as e:
            logger.error(f"Serial write error: {e}")

    def _serial_read_loop(self):
        """อ่านค่า ENCODER:<L>,<R> จาก Arduino #1"""
        while self._running:
            try:
                if self._ser.in_waiting == 0:
                    time.sleep(0.005)
                    continue
                line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("ENCODER:"):
                    parts = line[8:].split(",")
                    if len(parts) == 2:
                        l_ticks = int(parts[0])
                        r_ticks = int(parts[1])
                        self._update_odometry(l_ticks, r_ticks)
            except Exception:
                time.sleep(0.01)

    def _update_odometry(self, l_ticks: int, r_ticks: int):
        if self._first_enc:
            self._prev_l = l_ticks
            self._prev_r = r_ticks
            self._first_enc = False
            return

        # สลับขั้ว Encoder ซ้าย/ขวา แยกอิสระเพื่อแก้ปัญหาข้างใดข้างหนึ่งนับถอยหลัง
        sign_l = -1.0 if self._invert_left_enc else 1.0
        sign_r = -1.0 if self._invert_right_enc else 1.0
        sign_lin = -1.0 if self._invert_linear else 1.0

        dl = sign_lin * sign_l * (l_ticks - self._prev_l) * METERS_PER_TICK
        dr = sign_lin * sign_r * (r_ticks - self._prev_r) * METERS_PER_TICK
        self._prev_l = l_ticks
        self._prev_r = r_ticks

        d = (dl + dr) / 2.0
        d_theta = (dr - dl) / WHEEL_BASE
        if self._invert_angular:
            d_theta = -d_theta

        self._x += d * math.cos(self._theta + d_theta / 2.0)
        self._y += d * math.sin(self._theta + d_theta / 2.0)
        self._theta += d_theta

        # แสดง Log การขยับแบบเรียลไทม์ในเทอร์มินัลเมื่อล้อหมุน
        if abs(dl) > 0.0005 or abs(dr) > 0.0005:
            logger.info(
                f"[ODOM] dL={dl*100:+.1f}cm, dR={dr*100:+.1f}cm | "
                f"d={d*100:+.1f}cm, dTh={math.degrees(d_theta):+.1f}° | "
                f"Pos=({self._x:.2f}, {self._y:.2f})m Yaw={math.degrees(self._theta):.1f}°"
            )

    def _publish_odom_and_tf(self):
        now = self.get_clock().now().to_msg()
        half_theta = self._theta / 2.0
        qz = math.sin(half_theta)
        qw = math.cos(half_theta)

        transforms = []

        # 1. TF: odom -> base_footprint (สำหรับ slam_toolbox ที่ใช้ base_footprint)
        t_footprint = TransformStamped()
        t_footprint.header.stamp = now
        t_footprint.header.frame_id = "odom"
        t_footprint.child_frame_id = "base_footprint"
        t_footprint.transform.translation.x = self._x
        t_footprint.transform.translation.y = self._y
        t_footprint.transform.translation.z = 0.0
        t_footprint.transform.rotation.z = qz
        t_footprint.transform.rotation.w = qw
        transforms.append(t_footprint)

        # 2. TF: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.w = 1.0
        transforms.append(t_base)

        # 3. TF: base_link -> laser (และ laser_frame)
        half_yaw = self._yaw_offset / 2.0
        for child in ["laser", "laser_frame"]:
            t_laser = TransformStamped()
            t_laser.header.stamp = now
            t_laser.header.frame_id = "base_link"
            t_laser.child_frame_id = child
            t_laser.transform.translation.x = 0.15
            t_laser.transform.translation.y = 0.0
            t_laser.transform.translation.z = 0.10
            t_laser.transform.rotation.z = math.sin(half_yaw)
            t_laser.transform.rotation.w = math.cos(half_yaw)
            transforms.append(t_laser)

        for tr in transforms:
            self._tf_broadcaster.sendTransform(tr)

        # 4. Publish /odom Topic
        odom = OdomMsg()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self._odom_pub.publish(odom)


def main():
    if not HAS_ROS2:
        print("ERROR: ROS 2 (rclpy) is not available. Please source /opt/ros/jazzy/setup.bash first.")
        sys.exit(1)

    rclpy.init()

    # ค้นหาพอร์ต Arduino #1
    port = os.environ.get("MOTION_PORT", MOTION_SERIAL_PORT)
    ser = None
    if serial and port and port.lower() != "none":
        for p in [port, "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]:
            if os.path.exists(p):
                try:
                    ser = serial.Serial(p, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
                    logger.info(f"Opened Arduino Motion Port on {p}")
                    break
                except Exception as e:
                    logger.warning(f"Could not open {p}: {e}")

    yaw_offset = float(os.environ.get("LIDAR_YAW_OFFSET", "0.0"))
    node = SlamBridgeNode(ser=ser, yaw_offset_deg=yaw_offset)

    print("\n" + "=" * 55)
    print("  🗺️ SLAM & Teleop Test Bridge Running")
    print(f"  LiDAR Yaw Offset: {yaw_offset}°")
    print(f"  Motor Control   : {'Active via Arduino' if ser else 'Pure Laser / Mock'}")
    print("=" * 55)
    print("  พร้อมให้ SLAM Toolbox และ Teleop Keyboard เชื่อมต่อแล้ว\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        if ser:
            try:
                ser.write(b"V:0.000,0.000\n")
                ser.flush()
                ser.close()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()

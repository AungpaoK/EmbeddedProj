#!/usr/bin/env python3
"""
auto_explorer.py — Autonomous Exploration & Map-Building Node
=============================================================
ระบบนำทางสำรวจพื้นที่อัตโนมัติเพื่อสร้างแผนที่ SLAM 2D รอบห้อง/ร้านอาหาร

คุณสมบัติ:
  - ขับสำรวจพื้นที่ว่างอัตโนมัติโดยไม่ต้องใช้ Joy/Teleop
  - ระบบค้นหาช่องเปิด (Corridor & Open-Space Seeking)
  - ระบบหลบหลีกสิ่งกีดขวางแบบ Reactive 360° จาก RPLiDAR
  - ระบบแก้ทางตันด้วยการหมุนหาทิศโล่ง (Stuck & Dead-End Recovery)
  - ควบคุมความเร็วอย่างนุ่มนวล ป้องกันการลื่นไถลเพื่อรักษาคุณภาพ Odometry
"""

import os
import sys
import math
import time
import logging
from enum import Enum

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("AutoExplorer")


class ExplorerState(Enum):
    CRUISE = 1     # วิ่งสำรวจไปข้างหน้าในทิศทางที่โล่ง
    STEER = 2      # เลี้ยวปรับทิศทางตามช่องว่าง
    ESCAPE = 3     # หมุนหาทิศโล่งเมื่อเจอมุมตัน โดยไม่ถอยหลัง
    ROTATE_SCAN = 4  # หมุนตัว 360° ที่จุดเปิดเพื่อกวาดเก็บรายละเอียดแผนที่


class AutoExplorer(Node):
    def __init__(self):
        super().__init__("auto_explorer_node")

        # พารามิเตอร์ความเร็วและระยะปลอดภัย (ปรับแต่งผ่าน Environment Variables ได้)
        self.cruise_speed = float(os.environ.get("CRUISE_SPEED", "0.20"))      # m/s
        self.turn_speed = float(os.environ.get("TURN_SPEED", "0.55"))          # rad/s
        self.chassis_clearance = float(os.environ.get("CHASSIS_CLEARANCE", "0.22")) # เมตร: ตัดจุดสะท้อนเสา/โครงสร้างตัวถังด้านใน (เสาอยู่ที่ ~0.17m)
        self.emergency_dist = float(os.environ.get("EMERGENCY_DIST", "0.32"))  # เมตร: ถอยหลังทันทีถ้าประชิดเกินไป (> clearance)
        self.front_stop_dist = float(os.environ.get("FRONT_STOP_DIST", "0.45"))# เมตร: เริ่มหยุด/เลี้ยวเมื่อด้านหน้าใกล้กว่านี้
        self.side_min_dist = float(os.environ.get("SIDE_MIN_DIST", "0.28"))    # เมตร: ระยะกันชนด้านข้าง
        self.lidar_yaw_offset = math.radians(float(os.environ.get("LIDAR_YAW_OFFSET", "0.0")))

        # State Machine
        self.state = ExplorerState.CRUISE
        self.escape_end_time = 0.0
        self.escape_turn_speed = 0.0
        self.rotate_end_time = 0.0

        # Odometry Progress Watchdog (ตรวจจับกรณีหุ่นยนต์ติดขัด)
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_progress_time = time.time()
        self.current_x = 0.0
        self.current_y = 0.0

        # Stop if sensor data goes stale while exploring autonomously.
        self.last_scan_time = 0.0
        self.last_scan_warning_time = 0.0

        # รอบการหมุน 360° เพื่อเปิดแมป
        self.last_full_scan_time = time.time()
        self.scan_interval = 25.0       # ทุกๆ 25 วินาทีให้หยุดหมุนรอบตัว 360° กวาดแมป

        # ROS 2 Subscriptions & Publications
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_topic = os.environ.get("SCAN_TOPIC", "/scan_filtered")
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

        # Loop ประมวลผลควบคุมที่ 10 Hz
        self.timer = self.create_timer(0.1, self._control_loop)

        # เก็บผลวิเคราะห์ LaserScan ล่าสุด
        self.latest_scan_valid = False
        self.front_dist = float("inf")
        self.front_angle_deg = 0.0
        self.front_left_dist = float("inf")
        self.front_right_dist = float("inf")
        self.left_dist = float("inf")
        self.right_dist = float("inf")
        self.widest_direction = 0.0  # มุมที่เปิดโล่งที่สุด (-pi ถึง +pi)

        logger.info(
            f"Auto Explorer initialized: Clearance={self.chassis_clearance:.2f}m, "
            f"Emergency={self.emergency_dist:.2f}m, Stop={self.front_stop_dist:.2f}m, "
            f"Cruise={self.cruise_speed:.2f}m/s, Scan={self.scan_topic}"
        )

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # ตรวจสอบว่าเคลื่อนที่จริงหรือไม่ (ป้องกันติดขัด / ล้อฟรี)
        dist_moved = math.hypot(self.current_x - self.last_pos_x, self.current_y - self.last_pos_y)
        if dist_moved > 0.08:
            self.last_pos_x = self.current_x
            self.last_pos_y = self.current_y
            self.last_progress_time = time.time()

    def _scan_callback(self, msg: LaserScan):
        self.last_scan_time = time.monotonic()
        ranges = msg.ranges
        num_points = len(ranges)
        if num_points == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # ตัวแปรจำแนกโซนระยะทาง (ตัดจุดสะท้อน < chassis_clearance)
        zone_front = []
        zone_fl = []
        zone_fr = []
        zone_left = []
        zone_right = []

        sectors = [[] for _ in range(8)]  # แบ่ง 8 ทิศเพื่อหาช่องเปิดกว้างที่สุด

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r < self.chassis_clearance:
                continue

            angle = angle_min + i * angle_inc + self.lidar_yaw_offset
            # Normalize angle เป็น -pi ถึง +pi
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            # แบ่งโซนด้านหน้าและด้านข้าง
            if -25.0 <= deg <= 25.0:
                zone_front.append((r, deg))
            elif 25.0 < deg <= 65.0:
                zone_fl.append(r)
            elif -65.0 <= deg < -25.0:
                zone_fr.append(r)
            elif 65.0 < deg <= 115.0:
                zone_left.append(r)
            elif -115.0 <= deg < -65.0:
                zone_right.append(r)

            # วิเคราะห์ 8 ทิศทางรอบตัวเพื่อหาทางเปิด
            sector_idx = int((deg + 180.0) / 45.0) % 8
            sectors[sector_idx].append(r)

        # คำนวณระยะเฉลี่ย/ขั้นต่ำของแต่ละโซน
        front_hit = min(zone_front, key=lambda hit: hit[0]) if zone_front else (10.0, 0.0)
        self.front_dist, self.front_angle_deg = front_hit
        self.front_left_dist = min(zone_fl) if zone_fl else 10.0
        self.front_right_dist = min(zone_fr) if zone_fr else 10.0
        self.left_dist = min(zone_left) if zone_left else 10.0
        self.right_dist = min(zone_right) if zone_right else 10.0

        # หา Sector ที่ลึกและเปิดโล่งที่สุดในครึ่งวงหน้า (-90° ถึง +90°)
        best_dist = 0.0
        best_angle = 0.0
        for i, s in enumerate(sectors):
            avg_dist = (sum(s) / len(s)) if s else 0.0
            sector_center_deg = (i * 45.0) - 180.0 + 22.5
            # ให้ความสำคัญกับทางข้างหน้า (-80° ถึง +80°)
            if -80.0 <= sector_center_deg <= 80.0:
                if avg_dist > best_dist:
                    best_dist = avg_dist
                    best_angle = math.radians(sector_center_deg)

        self.widest_direction = best_angle
        self.latest_scan_valid = True

    def _start_escape(self, now: float):
        """Rotate toward the clearer side, keeping the robot's forward axis convention."""
        self.state = ExplorerState.ESCAPE
        turn_dir = 1.0 if self.left_dist > self.right_dist else -1.0
        self.escape_turn_speed = turn_dir * self.turn_speed
        self.escape_end_time = now + 2.2

    def _control_loop(self):
        now_monotonic = time.monotonic()
        if not self.latest_scan_valid or now_monotonic - self.last_scan_time > 0.5:
            # Never keep driving with an old scan if LiDAR or DDS drops out.
            self.cmd_pub.publish(Twist())
            if (
                self.latest_scan_valid
                and now_monotonic - self.last_scan_warning_time >= 2.0
            ):
                logger.warning("Laser scan %s is stale; publishing stop command", self.scan_topic)
                self.last_scan_warning_time = now_monotonic
            return

        now = time.time()
        cmd = Twist()

        # ------------------------------------------------------------------
        # 0. Watchdog ตรวจจับหุ่นติดขัด (Stuck Recovery)
        # ------------------------------------------------------------------
        if self.state == ExplorerState.CRUISE and (now - self.last_progress_time > 4.5):
            logger.warning("Stuck detected! Initiating escape maneuver...")
            self._start_escape(now)

        # ------------------------------------------------------------------
        # 1. หมุนตัว 360° ทุกระยะเวลาเพื่อกวาดแผนที่ให้เต็มห้อง
        # ------------------------------------------------------------------
        if self.state == ExplorerState.CRUISE and (now - self.last_full_scan_time > self.scan_interval):
            if self.front_dist > 0.8:  # ทำเฉพาะเมื่ออยู่ในที่โล่ง
                logger.info("Executing 360° panoramic scan for SLAM mapping...")
                self.state = ExplorerState.ROTATE_SCAN
                self.rotate_end_time = now + (2.0 * math.pi / self.turn_speed)
                self.last_full_scan_time = now

        # ------------------------------------------------------------------
        # 2. State Machine การนำทางสำรวจ
        # ------------------------------------------------------------------
        if self.state == ExplorerState.ROTATE_SCAN:
            if now < self.rotate_end_time:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed
            else:
                self.state = ExplorerState.CRUISE

        elif self.state == ExplorerState.ESCAPE:
            # Keep linear.x at zero during recovery. Cruise/steer use positive
            # linear.x, matching the forward command from teleop key 'i'.
            if now < self.escape_end_time:
                cmd.linear.x = 0.0
                cmd.angular.z = self.escape_turn_speed
            else:
                self.state = ExplorerState.CRUISE
                self.last_progress_time = now

        elif self.state == ExplorerState.STEER:
            # เลี้ยวปรับมุมหาช่องเปิด
            turn_bias = 1.0 if self.front_left_dist > self.front_right_dist else -1.0
            cmd.linear.x = 0.05
            cmd.angular.z = turn_bias * self.turn_speed

            if self.front_dist > self.front_stop_dist + 0.15:
                self.state = ExplorerState.CRUISE

        elif self.state == ExplorerState.CRUISE:
            # ตรวจสอบระยะฉุกเฉิน
            if self.front_dist < self.emergency_dist:
                logger.warning(
                    f"Emergency close distance ({self.front_dist:.2f}m at "
                    f"{self.front_angle_deg:+.1f}°)! Escaping..."
                )
                self._start_escape(now)
            elif self.front_dist < self.front_stop_dist:
                # ข้างหน้าเริ่มติด เลี้ยวหาช่องว่าง
                self.state = ExplorerState.STEER
            else:
                # ข้างหน้าโล่ง วิ่งไปข้างหน้าพร้อมเบี่ยงพวงมาลัยเข้าหาช่องกว้าง
                # ชะลอความเร็วตามระยะทางข้างหน้า
                speed_ratio = min(1.0, max(0.4, (self.front_dist - self.front_stop_dist) / 1.0))
                cmd.linear.x = self.cruise_speed * speed_ratio

                # เลี้ยวปรับหาช่องกว้างอย่างนุ่มนวล
                # มีแรงผลักจากกำแพงด้านข้าง (Wall Repulsion) ป้องกันชนขอบ
                side_repulsion = 0.0
                if self.left_dist < self.side_min_dist:
                    side_repulsion -= 0.35
                if self.right_dist < self.side_min_dist:
                    side_repulsion += 0.35

                cmd.angular.z = (0.6 * self.widest_direction) + side_repulsion
                cmd.angular.z = max(-self.turn_speed, min(self.turn_speed, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    def stop(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)


def main():
    if not HAS_ROS2:
        print("ERROR: ROS 2 is not available in this environment.")
        return

    rclpy.init()
    node = AutoExplorer()

    print("\n" + "=" * 60)
    print("  🚀 Auto Explorer: Autonomous 2D SLAM Mapping Running")
    print("=" * 60)
    print("  - ระบบกำลังสำรวจพื้นที่ วิ่งหาช่องเปิด และหลบสิ่งกีดขวาง")
    print("  - จะหยุดหมุน 360° ทุกๆ 25 วินาที เพื่อเก็บรายละเอียดแผนที่")
    print("  - กด Ctrl + C ได้ทุกเมื่อเพื่อหยุดหุ่นยนต์")
    print("=" * 60 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Stopping Auto Explorer...")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

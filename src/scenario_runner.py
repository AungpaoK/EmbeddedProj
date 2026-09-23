#!/usr/bin/env python3
"""
scenario_runner.py — Restaurant Delivery Scenario Runner & Simulator
===================================================================
รันสถานการณ์จำลองร้านอาหารตามข้อกำหนดใน docs/scenario.md:
  - Scenario 1: Single Table Delivery (ส่งโต๊ะ 1 แล้วกลับครัว)
  - Scenario 2: Multi-Table Delivery  (ส่งโต๊ะ 1 -> โต๊ะ 2 -> กลับครัว)

ฟังก์ชันการทำงาน:
  1. โหลดและ Publish แผนที่ร้านอาหาร (/map) จาก maps/restaurant_map.yaml
  2. แสดง 3D Markers โต๊ะ 1, โต๊ะ 2, ครัว (Serve Station) และเส้นทางใน RViz2
  3. รองรับ 2 โหมด:
     - Simulation Mode (--sim)  : จำลองการวิ่งแบบฟิสิกส์ 2D พร้อมสแกน LiDAR และ TF ใน RViz
     - Robot Mode      (--robot): สั่งงานหุ่นยนต์จริงผ่าน /cmd_vel และอ่านพิกัดจาก Odometry/SLAM
"""

import os
import sys
import math
import time
import argparse
import threading
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Point
    from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import ColorRGBA
    import tf2_ros
    from PIL import Image
    import yaml
    HAS_ROS2 = True
except ImportError as e:
    HAS_ROS2 = False
    ROS2_IMPORT_ERROR = str(e)

from config import (
    ARRIVAL_TOLERANCE_M,
    JUNCTION_X,
    TABLE1_Y,
    TABLE2_Y,
    WHEEL_BASE,
)

# พิกัดตาม docs/scenario.md
WAYPOINTS = {
    "kitchen":  (0.0, 0.0, 0.0),           # ครัว (Serve Station) หันหน้า 0°
    "junction": (JUNCTION_X, 0.0, 0.0),     # ทางแยก
    "table_1":  (JUNCTION_X, TABLE1_Y, 90.0), # โต๊ะ 1 (ซ้าย) หันหน้า +90°
    "table_2":  (JUNCTION_X, -TABLE2_Y, -90.0),# โต๊ะ 2 (ขวา) หันหน้า -90°
}


class ScenarioRunnerNode(Node):
    def __init__(self, mode: str = "sim", scenario: int = 1):
        super().__init__("scenario_runner_node")
        self.mode = mode
        self.scenario_id = scenario

        # สถานะตำแหน่งหุ่นยนต์ (x, y, theta_rad)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_pose = (0.0, 0.0, 0.0)
        self.target_v = 0.0
        self.target_w = 0.0

        # Odometry statistics & heartbeat
        self.odom_count = 0
        self.last_odom_time = 0.0

        # ROS 2 Publishers & Broadcasters
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/scenario_markers", latched_qos)
        self.path_pub = self.create_publisher(Path, "/robot_path", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # แยกความรับผิดชอบระหว่าง Sim Mode และ Real Robot Mode
        if self.mode == "robot":
            self.odom_sub = self.create_subscription(Odometry, "/odom", self._real_odom_callback, 10)
            self.odom_pub = None
            self.scan_pub = None
        else:
            self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.odom_sub = None

        # เส้นทางสะสม (Trail)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # โหลดและ Publish แผนที่
        self.map_grid = self._load_occupancy_grid()
        if self.map_grid:
            self.map_pub.publish(self.map_grid)
            self._broadcast_static_tf()
            self._publish_scenario_markers()

        # Timer จำลองและอัปเดตสถานะที่ 20 Hz
        self.timer = self.create_timer(0.05, self._simulation_step)

        # Timer รีเฟรช Map, Markers และ TF ทุก 1 วินาที เพื่อให้ RViz2 ที่เปิดทีหลังได้รับข้อมูลทันที
        self.map_timer = self.create_timer(1.0, self._periodic_map_publish)

        # Thread รันสถานการณ์จำลองตาม docs/scenario.md
        self.running = True
        self.mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self.mission_thread.start()

    def _load_occupancy_grid(self) -> OccupancyGrid:
        """อ่านไฟล์ PGM และ YAML เพื่อสร้าง nav_msgs/OccupancyGrid"""
        maps_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "maps")
        yaml_path = os.path.join(maps_dir, "restaurant_map.yaml")
        
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"ไม่พบไฟล์แผนที่: {yaml_path}")
            return None

        with open(yaml_path, "r") as f:
            yaml_data = yaml.safe_load(f)

        pgm_name = yaml_data["image"]
        pgm_path = os.path.join(maps_dir, pgm_name)
        if not os.path.exists(pgm_path):
            self.get_logger().error(f"ไม่พบไฟล์ภาพ PGM: {pgm_path}")
            return None

        img = Image.open(pgm_path)
        width, height = img.size
        resolution = float(yaml_data["resolution"])
        origin = yaml_data["origin"]

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = float(origin[2])
        grid.info.origin.orientation.w = 1.0

        data = []
        # แปลงข้อมูลภาพ: PIL y=0 คือขอบบน แต่ ROS grid y=0 คือขอบล่าง
        for y in reversed(range(height)):
            for x in range(width):
                val = img.getpixel((x, y))
                if val == 254:
                    data.append(0)    # Free
                elif val == 0:
                    data.append(100)  # Occupied (Wall/Table)
                else:
                    data.append(-1)   # Unknown
        grid.data = data
        self.get_logger().info(f"Loaded map: {width}x{height} pixels ({resolution}m/px)")
        return grid

    def _broadcast_static_tf(self):
        """Broadcast Static TF: map -> odom (และ base_link -> laser เฉพาะในโหมด sim)"""
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = "map"
        t_map.child_frame_id = "odom"
        t_map.transform.rotation.w = 1.0

        transforms = [t_map]
        if self.mode == "sim":
            t_laser = TransformStamped()
            t_laser.header.stamp = self.get_clock().now().to_msg()
            t_laser.header.frame_id = "base_link"
            t_laser.child_frame_id = "laser"
            t_laser.transform.translation.x = 0.15
            t_laser.transform.translation.z = 0.10
            t_laser.transform.rotation.w = 1.0
            transforms.append(t_laser)

        self.static_tf_broadcaster.sendTransform(transforms)

    def _publish_scenario_markers(self):
        """สร้าง 3D Visual Markers ใน RViz2 แสดง Kitchen, Junction, Table 1, Table 2"""
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        def make_box(idx, x, y, name, color, sx=0.8, sy=0.6, sz=0.7):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "map"
            m.ns = "tables"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = sz / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz
            m.color = color
            markers.markers.append(m)

            # ป้ายชื่อข้อความ 3D Text
            t = Marker()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.ns = "labels"
            t.id = idx + 100
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = sz + 0.35
            t.scale.z = 0.25
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = name
            markers.markers.append(t)

        # 1. Kitchen / Serve Station
        make_box(1, -0.7, 0.0, "🍳 Serve Station (Kitchen)", ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.85), sx=0.6, sy=1.2)
        # 2. Junction
        make_box(2, JUNCTION_X, 0.0, "✛ Junction", ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.4), sx=0.4, sy=0.4, sz=0.02)
        # 3. Table 1 (เหนือ / ซ้าย)
        make_box(3, JUNCTION_X, TABLE1_Y + 0.5, f"🍽️ Table 1 (+{TABLE1_Y}m)", ColorRGBA(r=1.0, g=0.8, b=0.1, a=0.9))
        # 4. Table 2 (ใต้ / ขวา)
        make_box(4, JUNCTION_X, -TABLE2_Y - 0.5, f"🍽️ Table 2 (-{TABLE2_Y}m)", ColorRGBA(r=1.0, g=0.4, b=0.1, a=0.9))

        self.marker_pub.publish(markers)

    def _periodic_map_publish(self):
        """รีเฟรช Map, Static TF และ Markers เป็นระยะ เพื่อให้ RViz2 ที่เปิดทีหลังเชื่อมต่อได้ทันที"""
        if self.map_grid:
            self.map_pub.publish(self.map_grid)
        self._broadcast_static_tf()
        self._publish_scenario_markers()

    def _real_odom_callback(self, msg: Odometry):
        self.odom_count += 1
        self.last_odom_time = time.time()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def _simulation_step(self):
        """Simulation Loop 20 Hz: จำลองจลนศาสตร์ และบรอดคาสต์ TF / Odom (เฉพาะโหมด sim) และอัปเดต Trail"""
        now = self.get_clock().now().to_msg()
        dt = 0.05

        if self.mode == "sim":
            # อัปเดตพิกัดด้วยสมการ Differential Drive
            self.x += self.target_v * math.cos(self.theta) * dt
            self.y += self.target_v * math.sin(self.theta) * dt
            self.theta += self.target_w * dt

            half_th = self.theta / 2.0
            qz = math.sin(half_th)
            qw = math.cos(half_th)

            # 1. TF: odom -> base_footprint -> base_link
            t_foot = TransformStamped()
            t_foot.header.stamp = now
            t_foot.header.frame_id = "odom"
            t_foot.child_frame_id = "base_footprint"
            t_foot.transform.translation.x = self.x
            t_foot.transform.translation.y = self.y
            t_foot.transform.rotation.z = qz
            t_foot.transform.rotation.w = qw

            t_base = TransformStamped()
            t_base.header.stamp = now
            t_base.header.frame_id = "base_footprint"
            t_base.child_frame_id = "base_link"
            t_base.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform([t_foot, t_base])

            # 2. Publish /odom
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = self.target_v
            odom.twist.twist.angular.z = self.target_w
            self.odom_pub.publish(odom)

            # 3. จำลองสแกน LiDAR (/scan) สะท้อนกำแพงในโหมด sim
            self._simulate_lidar_scan(now)

        # 4. อัปเดตเส้นทาง Path Trail ใน RViz (ทั้ง sim และ real robot)
        half_th = self.theta / 2.0
        qz = math.sin(half_th)
        qw = math.cos(half_th)
        if abs(self.target_v) > 0.01 or abs(self.target_w) > 0.05:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses.pop(0)
            self.path_pub.publish(self.path_msg)

    def _simulate_lidar_scan(self, now):
        """Raycasting แบบง่ายรอบตัว 360° จำลองระยะเสมือนจริง"""
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        num_readings = 360
        scan.angle_increment = (2.0 * math.pi) / num_readings
        scan.time_increment = 0.0003
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 12.0

        ranges = []
        for i in range(num_readings):
            ray_ang = self.theta + (scan.angle_min + i * scan.angle_increment)
            dist = 12.0
            # ตรวจระยะกำแพงกรอบนอกห้อง (X: -1.0 ถึง 3.4, Y: -1.8 ถึง 1.8)
            cos_a = math.cos(ray_ang)
            sin_a = math.sin(ray_ang)
            
            if cos_a > 1e-4:
                dist = min(dist, (3.4 - self.x) / cos_a)
            elif cos_a < -1e-4:
                dist = min(dist, (-1.0 - self.x) / cos_a)
            if sin_a > 1e-4:
                dist = min(dist, (1.8 - self.y) / sin_a)
            elif sin_a < -1e-4:
                dist = min(dist, (-1.8 - self.y) / sin_a)
            ranges.append(max(0.1, dist))

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    # ------------------------------------------------------------------
    # การควบคุมการเคลื่อนที่ตาม Waypoints (docs/scenario.md)
    # ------------------------------------------------------------------
    def drive_forward(
        self,
        distance: float,
        speed: float = 0.22,
        target_heading: Optional[float] = None,
    ) -> bool:
        """Drive a measured distance while correcting yaw drift from odometry."""
        print(f"  ⬆️ [Motion] เดินหน้า {distance:.2f} เมตร (ความเร็ว {speed:.2f} m/s)...")
        last_x, last_y = self.x, self.y
        if target_heading is None:
            target_heading = self.theta
        target_heading = self._wrap_angle(target_heading)
        traveled = 0.0

        max_duration = (distance / max(speed, 0.05)) * 2.5 + 5.0
        start_time = time.time()
        last_progress_time = time.time()
        last_traveled = 0.0
        last_heading_log_time = 0.0
        success = False

        while rclpy.ok() and traveled < distance:
            now_t = time.time()
            if now_t - start_time > max_duration:
                print(f"  ⚠️ [Timeout] เดินหน้าครบกำหนดเวลา ({max_duration:.1f}s) เดินได้ {traveled:.2f}/{distance:.2f}m")
                break

            if traveled - last_traveled > 0.01:
                last_progress_time = now_t
                last_traveled = traveled
            elif now_t - last_progress_time > 4.0:
                print("  ⚠️ [Warning] ไม่พบการเปลี่ยนแปลงตำแหน่งจาก /odom เกิน 4 วินาที! มอเตอร์อาจติดขัดหรือเซนเซอร์ไม่ทำงาน")
                last_progress_time = now_t

            # Hold the heading present at the start of this straight segment.
            # Positive correction is ROS/odom CCW; compensate for the bridge's
            # INVERT_STEER setting when publishing the real-robot command.
            heading_error = self._wrap_angle(target_heading - self.theta)
            correction = 0.0
            if abs(heading_error) > math.radians(1.0):
                correction = max(-0.5, min(0.5, 1.8 * heading_error))
                if abs(correction) < 0.12:
                    correction = math.copysign(0.12, correction)

            self.target_v = speed
            self.target_w = correction
            if self.mode == "robot":
                cmd = Twist()
                cmd.linear.x = speed
                cmd.angular.z = -correction
                self.cmd_pub.publish(cmd)

            if abs(heading_error) > math.radians(5.0) and now_t - last_heading_log_time > 1.0:
                print(
                    f"  ↪️ [Heading Hold] เบน {math.degrees(heading_error):+.1f}° "
                    f"กำลังชดเชยทิศทาง"
                )
                last_heading_log_time = now_t
            time.sleep(0.05)

            step_dist = math.hypot(self.x - last_x, self.y - last_y)
            traveled += step_dist
            last_x, last_y = self.x, self.y

        self.stop_robot()
        if traveled >= max(0.0, distance - ARRIVAL_TOLERANCE_M):
            success = True
            print(f"  ✓ [Motion] เดินหน้าสำเร็จ รวมระยะ {traveled:.2f}m (ตำแหน่งปัจจุบัน: X={self.x:.2f}, Y={self.y:.2f})")
        else:
            print(f"  ⚠️ [Motion] เดินหน้าไม่ครบระยะ (ได้ {traveled:.2f}/{distance:.2f}m)")
        return success

    def turn_degrees(self, degrees: float, speed: float = 0.75) -> bool:
        """Turn by a relative angle, retaining the closed-loop odometry check."""
        return self.turn_to_heading(self.theta + math.radians(degrees), speed=speed)

    def turn_to_heading(
        self,
        target_heading: float,
        speed: float = 0.75,
        tolerance_degrees: float = 2.5,
    ) -> bool:
        """Turn in place to an absolute odom heading, easing speed near target."""
        target_theta = self._wrap_angle(target_heading)
        initial_error = self._wrap_angle(target_theta - self.theta)
        direction = "ซ้าย (CCW)" if initial_error >= 0.0 else "ขวา (CW)"
        print(
            f"  🔄 [Motion] หมุน{direction} ไป heading "
            f"{math.degrees(target_theta):.1f}° ที่ความเร็วสูงสุด {speed:.2f} rad/s..."
        )

        max_duration = (abs(initial_error) / max(speed, 0.1)) * 3.0 + 6.0
        start_time = time.time()
        last_progress_time = time.time()
        last_err = 999.0
        success = False
        tolerance = math.radians(tolerance_degrees)
        slowdown_range = math.radians(45.0)
        minimum_turn_speed = min(speed, 0.60)

        while rclpy.ok():
            now_t = time.time()
            if now_t - start_time > max_duration:
                print(f"  ⚠️ [Timeout] หมุนครบกำหนดเวลา ({max_duration:.1f}s) ยังไม่ถึงเป้าหมาย (Yaw ปัจจุบัน: {math.degrees(self.theta):.1f}°)")
                break

            # คำนวณ error ของมุมในรอบ [-pi, +pi]
            err = self._wrap_angle(target_theta - self.theta)

            if abs(err) <= tolerance:
                # Let encoder updates catch any small coast after zero velocity,
                # then verify the final heading before allowing the next leg.
                self.stop_robot()
                time.sleep(0.15)
                settled_err = self._wrap_angle(target_theta - self.theta)
                if abs(settled_err) <= tolerance:
                    success = True
                    break
                print(
                    f"  ↪️ [Turn settle] หลังหยุดยังคลาด "
                    f"{math.degrees(settled_err):+.1f}° กำลังปรับซ้ำ"
                )
                last_err = 999.0
                last_progress_time = time.time()
                continue

            # Stall check: เช็คว่า error ขยับลดลงหรือไม่
            if abs(err - last_err) > math.radians(1.5):
                last_progress_time = now_t
                last_err = err
            elif now_t - last_progress_time > 4.0:
                print("  ⚠️ [Warning] ไม่พบการหมุนจาก /odom เกิน 4 วินาที! มอเตอร์อาจติดขัด")
                last_progress_time = now_t

            # Keep the tank-turn wheel targets strong enough to break away,
            # but ease down from max speed over the final 45 degrees.
            remaining = abs(err)
            if remaining >= slowdown_range:
                turn_speed = speed
            else:
                progress = max(
                    0.0,
                    min(1.0, (remaining - tolerance) / (slowdown_range - tolerance)),
                )
                turn_speed = minimum_turn_speed + (speed - minimum_turn_speed) * progress

            turn_w = math.copysign(turn_speed, err)
            self.target_v = 0.0
            self.target_w = turn_w
            if self.mode == "robot":
                cmd = Twist()
                # Zero linear velocity makes the bridge command equal-magnitude,
                # opposite-sign wheel speeds for an in-place tank turn.
                cmd.linear.x = 0.0
                # The robot bridge uses INVERT_STEER=1 to preserve the
                # project's j/l teleop direction mapping. Scenario angles
                # follow ROS/odom convention (+ = CCW), so compensate here
                # to make the closed-loop command move yaw toward its target.
                cmd.angular.z = -turn_w
                self.cmd_pub.publish(cmd)
            time.sleep(0.05)

        self.stop_robot()
        if success:
            print(f"  ✓ [Motion] หมุนสำเร็จ (Yaw ปัจจุบัน: {math.degrees(self.theta):.1f}°)")
        else:
            print(f"  ⚠️ [Motion] การหมุนยังไม่ตรงเป้าหมาย (ได้ Yaw: {math.degrees(self.theta):.1f}°)")
        return success

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        """หยุดหุ่นยนต์และตัดกำลังขับเคลื่อน"""
        self.target_v = 0.0
        self.target_w = 0.0
        if rclpy.ok():
            try:
                cmd = Twist()
                for _ in range(3):
                    self.cmd_pub.publish(cmd)
                    time.sleep(0.05)
            except Exception:
                pass

    def wait_customer_pickup(self, table_name: str, wait_sec: float = 3.0):
        """จำลองการรอลูกค้าหยิบอาหาร (IR Sensor / Manual Override)"""
        print(f"\n  🔔 [Service] ถึง {table_name} แล้ว! กะพริบไฟเบรกสีแดง รอลูกค้ารับอาหาร...")
        for remaining in range(int(wait_sec), 0, -1):
            print(f"     ⏳ กำลังรอลูกค้าหยิบถาดอาหาร... ({remaining}s)")
            time.sleep(1.0)
        print(f"  ✓ [Service] ลูกค้าหยิบอาหารเรียบร้อยแล้ว (IR Sensor Clear)\n")

    # ------------------------------------------------------------------
    # Scenario Flow Execution
    # ------------------------------------------------------------------
    def _run_mission(self):
        time.sleep(1.5)  # รอ RViz2 เชื่อมต่อ
        print("\n" + "=" * 60)
        print(f"  🤖 FOOD DELIVERY ROBOT — SCENARIO {self.scenario_id} RUNNER")
        print(f"  โหมดการทำงาน: {'🎮 SIMULATION (จำลอง 2D ใน RViz2)' if self.mode == 'sim' else '🚀 REAL ROBOT (สั่งหุ่นยนต์จริง)'}")
        print("=" * 60)

        if self.mode == "robot":
            print("  ⏳ [Hardware Preflight] กำลังตรวจสอบการเชื่อมต่อกับหุ่นยนต์ (/odom)...")
            wait_start = time.time()
            while rclpy.ok() and self.odom_count < 2:
                time.sleep(0.2)
                if time.time() - wait_start > 4.0:
                    print("\n  ⚠️ [Hardware Warning] ยังไม่ได้รับข้อมูลจาก /odom เกิน 4 วินาที!")
                    print("     คำแนะนำการตรวจสอบ:")
                    print("     1. slam_bridge.py กำลังทำงานอยู่บน Raspberry Pi หรือไม่")
                    print("     2. สาย USB ต่อเข้า Arduino #1 เสียบแน่นหรือไม่ (/dev/ttyACM0)")
                    print("     (กำลังรอสัญญาณต่อไป... หากตรวจพบแล้วจะเริ่มปฏิบัติภารกิจทันที)\n")
                    wait_start = time.time()

            print(f"  ✓ [Hardware Connected] ตรวจพบข้อมูลจาก Arduino แล้ว! (x={self.x:.2f}, y={self.y:.2f}, th={math.degrees(self.theta):.1f}°)\n")

        # Use the actual odometry pose at mission start as the return target.
        self.start_pose = (self.x, self.y, self.theta)
        print(
            f"  📍 [Mission Start] X={self.x:.2f}, Y={self.y:.2f}, "
            f"Yaw={math.degrees(self.theta):.1f}°"
        )

        if self.scenario_id == 1:
            self._execute_scenario_1()
        elif self.scenario_id == 2:
            self._execute_scenario_2()
        else:
            print("Unknown scenario ID.")

    def _execute_scenario_1(self):
        """Scenario 1: Single Table Delivery (ส่ง Table 1 แล้วกลับครัว) พร้อมระบบป้องกันชนสิ่งกีดขวาง"""
        print("\n--- [Scenario 1] เริ่มต้นส่งอาหารโต๊ะ 1 ---")
        print("1. วางอาหารชั้น 1 กำหนดส่ง Table 1 -> กดยืนยันการออกส่ง (#)")
        time.sleep(1.0)

        start_heading = self.start_pose[2]
        heading_out = self._wrap_angle(start_heading)
        heading_table = self._wrap_angle(start_heading + math.pi / 2.0)
        heading_return = self._wrap_angle(start_heading - math.pi / 2.0)
        heading_kitchen = self._wrap_angle(start_heading + math.pi)

        # Fixed sequence: kitchen -> junction -> table -> junction -> kitchen.
        print(f"\n[Step 1/5] เดินตรงไปยัง Junction {JUNCTION_X:.2f}m...")
        if not self.drive_forward(JUNCTION_X, target_heading=heading_out):
            print("  ⚠️ [Safety Abort] การเดินหน้าขัดข้อง ยกเลิกขั้นตอนถัดไปเพื่อความปลอดภัย!")
            return

        print("\n[Step 2/5] หมุนเข้าหาโต๊ะ 1...")
        if not self.turn_to_heading(heading_table):
            print("  ⚠️ [Safety Abort] หมุนเข้าหาโต๊ะไม่สำเร็จ")
            return

        print(f"\n[Step 3/5] เดินตรงไปที่โต๊ะ {TABLE1_Y:.2f}m...")
        if not self.drive_forward(TABLE1_Y, target_heading=heading_table):
            print("  ⚠️ [Safety Abort] การเข้าเทียบโต๊ะขัดข้อง ยกเลิกขั้นตอนถัดไปเพื่อความปลอดภัย!")
            return

        self.wait_customer_pickup("Table 1", wait_sec=3.5)

        print("[Step 4/5] หมุนกลับ แล้วเดินตรงกลับมายัง Junction...")
        if not self.turn_to_heading(heading_return):
            print("  ⚠️ [Safety Abort] หมุนกลับจากโต๊ะไม่สำเร็จ")
            return
        if not self.drive_forward(TABLE1_Y, target_heading=heading_return):
            print("  ⚠️ [Safety Abort] การวิ่งกลับทางแยกขัดข้อง ยกเลิกขั้นตอนถัดไปเพื่อความปลอดภัย!")
            return

        print("[Step 5/5] หมุนเข้าหาครัว แล้วเดินตรงกลับจุดเริ่ม...")
        if not self.turn_to_heading(heading_kitchen):
            print("  ⚠️ [Safety Abort] หมุนเข้าหาครัวไม่สำเร็จ")
            return
        if not self.drive_forward(JUNCTION_X, target_heading=heading_kitchen):
            print("  ⚠️ [Safety Abort] การเดินกลับครัวขัดข้อง!")
            return

        print("\n" + "=" * 60)
        start_x, start_y, _ = self.start_pose
        return_offset = math.hypot(self.x - start_x, self.y - start_y)
        print(f"  📍 [Return Check] คลาดจากจุดเริ่มตาม odom {return_offset:.2f}m")
        if return_offset > ARRIVAL_TOLERANCE_M:
            print("  ⚠️ [SCENARIO 1 FINISHED WITH POSITION ERROR] ถึงครัวแต่ยังคลาดจากจุดเริ่ม")
        else:
            print("  🎉 [SCENARIO 1 COMPLETED] ส่งอาหารโต๊ะ 1 สำเร็จ จอดพร้อมรับงานรอบใหม่!")
        print("=" * 60 + "\n")

    def _execute_scenario_2(self):
        """Scenario 2: Multi-Table Delivery (ส่ง Table 1 -> Table 2 -> กลับครัว)"""
        print("\n--- [Scenario 2] เริ่มต้นส่งอาหาร 2 โต๊ะพร้อมกัน (Table 1 & Table 2) ---")
        print("1. ชั้น 1: Table 1 | ชั้น 2: Table 2 -> กดยืนยัน (#)")
        time.sleep(1.0)

        start_heading = self.start_pose[2]
        heading_out = self._wrap_angle(start_heading)
        heading_table1 = self._wrap_angle(start_heading + math.pi / 2.0)
        heading_table2 = self._wrap_angle(start_heading - math.pi / 2.0)
        heading_kitchen = self._wrap_angle(start_heading + math.pi)

        # --- Deliver Table 1 ---
        print("\n>>> ส่งโต๊ะที่ 1 (Table 1) <<<")
        if not self.drive_forward(JUNCTION_X, target_heading=heading_out): return
        if not self.turn_to_heading(heading_table1): return
        if not self.drive_forward(TABLE1_Y, target_heading=heading_table1): return
        self.wait_customer_pickup("Table 1 (ชั้น 1)", wait_sec=3.0)

        # --- Deliver Table 2 ---
        print("\n>>> เดินทางไปส่งโต๊ะที่ 2 (Table 2) <<<")
        if not self.turn_to_heading(heading_table2): return
        if not self.drive_forward(TABLE1_Y, target_heading=heading_table2): return
        if not self.drive_forward(TABLE2_Y, target_heading=heading_table2): return
        self.wait_customer_pickup("Table 2 (ชั้น 2)", wait_sec=3.0)

        # --- Return to Kitchen ---
        print("\n>>> ส่งครบทั้ง 2 โต๊ะแล้ว เดินทางกลับครัว <<<")
        if not self.turn_to_heading(heading_table1): return
        if not self.drive_forward(TABLE2_Y, target_heading=heading_table1): return
        if not self.turn_to_heading(heading_kitchen): return
        if not self.drive_forward(JUNCTION_X, target_heading=heading_kitchen): return

        print("\n" + "=" * 60)
        start_x, start_y, _ = self.start_pose
        return_offset = math.hypot(self.x - start_x, self.y - start_y)
        print(f"  📍 [Return Check] คลาดจากจุดเริ่มตาม odom {return_offset:.2f}m")
        if return_offset > ARRIVAL_TOLERANCE_M:
            print("  ⚠️ [SCENARIO 2 FINISHED WITH POSITION ERROR] ถึงครัวแต่ยังคลาดจากจุดเริ่ม")
        else:
            print("  🎉 [SCENARIO 2 COMPLETED] เสิร์ฟครบ 2 โต๊ะ และเดินทางกลับครัวเรียบร้อย!")
        print("=" * 60 + "\n")


def main():
    if not HAS_ROS2:
        print(f"ERROR: ROS 2 is not available: {ROS2_IMPORT_ERROR}")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Restaurant Delivery Scenario Runner & Simulator")
    parser.add_argument("--sim", action="store_true", default=True, help="Run in pure simulation mode (default)")
    parser.add_argument("--robot", action="store_true", help="Run on real physical robot via /cmd_vel")
    parser.add_argument("--scenario", type=int, default=1, choices=[1, 2], help="Scenario number: 1 or 2 (default: 1)")
    args = parser.parse_args()

    mode = "robot" if args.robot else "sim"

    rclpy.init()
    node = ScenarioRunnerNode(mode=mode, scenario=args.scenario)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.running = False
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

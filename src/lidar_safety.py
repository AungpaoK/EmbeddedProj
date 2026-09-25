#!/usr/bin/env python3
"""
lidar_safety.py — LiDAR Safety Guard (Virtual Bumper)
=====================================================
ตรวจสอบข้อมูล LaserScan เพื่อป้องกันการชนและหยุดรถเมื่อพบคนหรือสิ่งกีดขวาง

คุณสมบัติ:
  - รองรับการตั้งค่า Yaw Offset (หมุนองศา LiDAR ให้ตรงกับหน้ารถ base_link)
  - กรองมุมบอดของโครงสร้างตัวถังหุ่นยนต์ (Self-Occlusion Filter)
  - กำหนด Safety Cone ด้านหน้า (เช่น ±35°, ระยะ < 0.50m)
  - กรอง Noise / จุดสะท้อนเดี่ยว (Speckle Filter)
"""

import math
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class LidarSafetyGuard:
    """
    ตรวจสอบข้อมูล LaserScan ในโซนปลอดภัยด้านหน้าหุ่นยนต์
    """

    def __init__(
        self,
        stop_distance_m: float = 0.30,
        front_cone_deg: float = 35.0,
        min_clearance_m: float = 0.22,  # กรองแผ่นตัวถังหรือเสาโครงสร้างด้านใน (~0.17m)
        yaw_offset_deg: float = 0.0,    # องศาชดเชยการยึด LiDAR เทียบกับหน้ารถ
        min_obstacle_points: int = 3,   # ต้องเจออย่างน้อย 3 จุดติดกันเพื่อตัด Noise
    ) -> None:
        self.stop_distance_m = stop_distance_m
        self.front_cone_deg = front_cone_deg
        self.min_clearance_m = min_clearance_m
        self.yaw_offset_deg = yaw_offset_deg
        self.min_obstacle_points = min_obstacle_points

        self._obstacle_detected: bool = False
        self._closest_distance: float = float("inf")
        self._closest_angle_deg: float = 0.0

    @property
    def is_obstacle_detected(self) -> bool:
        """True หากพบคนหรือสิ่งกีดขวางใน Safety Zone ด้านหน้า"""
        return self._obstacle_detected

    @property
    def closest_distance(self) -> float:
        """ระยะห่างวัตถุที่ใกล้ที่สุด (เมตร)"""
        return self._closest_distance

    def update_scan(
        self,
        ranges: list[float],
        angle_min: float,
        angle_max: float,
        angle_increment: float,
        range_min: float = 0.05,
        range_max: float = 12.0,
    ) -> bool:
        """
        ประมวลผลข้อมูลการสแกน (ใช้ได้ทั้งกับ ROS 2 LaserScan หรือ Raw Data)
        """
        obstacle_count = 0
        min_dist = float("inf")
        min_angle = 0.0

        yaw_offset_rad = math.radians(self.yaw_offset_deg)
        half_cone_rad = math.radians(self.front_cone_deg)

        for i, dist in enumerate(ranges):
            # ตรวจสอบค่าระยะที่สมเหตุสมผล
            if math.isnan(dist) or math.isinf(dist):
                continue
            if dist < max(range_min, self.min_clearance_m) or dist > range_max:
                continue

            # คำนวณมุมของลำแสงนี้ เทียบกับหน้ารถ (Robot Forward = 0 rad)
            raw_angle = angle_min + i * angle_increment
            robot_angle = raw_angle + yaw_offset_rad

            # ปรับมุมให้อยู่ในช่วง [-π, +π]
            robot_angle = (robot_angle + math.pi) % (2.0 * math.pi) - math.pi

            # ตรวจสอบว่าอยู่ในกรวยด้านหน้าหรือไม่ (-half_cone <= angle <= +half_cone)
            if abs(robot_angle) <= half_cone_rad:
                if dist <= self.stop_distance_m:
                    obstacle_count += 1
                    if dist < min_dist:
                        min_dist = dist
                        min_angle = math.degrees(robot_angle)

        if obstacle_count >= self.min_obstacle_points:
            if not self._obstacle_detected:
                logger.warning(
                    f"[LiDAR Safety] OBSTACLE DETECTED at {min_dist:.2f}m ({min_angle:.1f}°)"
                )
            self._obstacle_detected = True
            self._closest_distance = min_dist
            self._closest_angle_deg = min_angle
        else:
            if self._obstacle_detected:
                logger.info("[LiDAR Safety] Obstacle cleared. Path is open.")
            self._obstacle_detected = False
            self._closest_distance = float("inf")
            self._closest_angle_deg = 0.0

        return self._obstacle_detected

    def ros2_scan_callback(self, msg) -> None:
        """
        Callback ฟังก์ชันสำหรับเชื่อมต่อกับ ROS 2 LaserScan message
        """
        self.update_scan(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )

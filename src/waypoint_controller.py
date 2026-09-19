#!/usr/bin/env python3
"""
waypoint_controller.py — Closed-Loop Continuous Waypoint Controller
===================================================================
ควบคุมการเคลื่อนที่ไปยังพิกัดเป้าหมาย (x, y, theta) แบบปิดรอบ (Closed-Loop)
โดยคำนวณและสตรีมความเร็ว v, w ลง Arduino #1 อย่างต่อเนื่อง

รองรับ:
  - การรับพิกัดจาก SLAM Pose หรือ Wheel Odometry
  - ระบบหยุดฉุกเฉินเมื่อเจอคน/สิ่งกีดขวาง (เชื่อมกับ LidarSafetyGuard)
  - Ramping ความเร็วเพื่อไม่ให้อาหารกระฉอก
"""

import math
import time
import logging
from typing import Callable, Optional, Tuple, TYPE_CHECKING

from config import (
    WHEEL_BASE,
    ARRIVAL_TOLERANCE_M,
    HEADING_TOLERANCE_DEG,
)
if TYPE_CHECKING:
    from motion_client import MotionClient
from lidar_safety import LidarSafetyGuard

logger = logging.getLogger(__name__)


def normalize_angle(angle_rad: float) -> float:
    """ปรับมุมให้อยู่ในช่วง [-π, +π]"""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class WaypointController:
    """
    Closed-loop Controller นำทางหุ่นยนต์ไปยังเป้าหมาย Waypoint
    """

    def __init__(
        self,
        motion: MotionClient,
        safety_guard: LidarSafetyGuard,
        pose_provider: Callable[[], Tuple[float, float, float]],
        max_linear_speed: float = 0.25,   # m/s
        min_linear_speed: float = 0.05,   # m/s
        max_angular_speed: float = 0.60,  # rad/s
        control_rate_hz: float = 15.0,    # 15 Hz streaming rate
    ) -> None:
        self._motion = motion
        self._safety = safety_guard
        self._get_pose = pose_provider

        self.max_linear_speed = max_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_speed = max_angular_speed
        self.control_period = 1.0 / control_rate_hz

        # Controller Gains
        self.kp_dist = 0.6
        self.kp_angle = 1.5

        self._active = False

    def navigate_to(
        self,
        target_x: float,
        target_y: float,
        target_theta_deg: Optional[float] = None,
        timeout_s: float = 45.0,
    ) -> bool:
        """
        นำทางหุ่นยนต์ไปยัง (target_x, target_y) และจัดมุมหัว (target_theta_deg)
        คืนค่า True เมื่อถึงเป้าหมาย, False หากหมดเวลา
        """
        logger.info(
            f"[Nav] Navigating to ({target_x:.2f}, {target_y:.2f})"
            + (f", {target_theta_deg:.1f}°" if target_theta_deg is not None else "")
        )
        self._active = True
        start_time = time.monotonic()

        while self._active:
            if time.monotonic() - start_time > timeout_s:
                logger.error(f"[Nav] Timeout reaching ({target_x}, {target_y})")
                self._motion.stop_continuous()
                self._active = False
                return False

            # 1. อ่านพิกัดปัจจุบัน (x, y, theta_rad)
            x, y, theta_rad = self._get_pose()

            # 2. ตรวจสอบ LiDAR Safety Bumper
            if self._safety.is_obstacle_detected:
                # สั่งหยุดมอเตอร์ทันที รอจนกว่าคนจะเดินพ้น
                self._motion.stop_continuous()
                time.sleep(self.control_period)
                continue

            # 3. คำนวณระยะห่างถึงเป้าหมาย
            dx = target_x - x
            dy = target_y - y
            distance_err = math.hypot(dx, dy)

            # 4. กรณีที่ยังไม่ถึงจุดหมาย (Position Phase)
            if distance_err > ARRIVAL_TOLERANCE_M:
                # มุมเป้าหมายที่ต้องมุ่งหน้าไป
                target_heading_rad = math.atan2(dy, dx)
                heading_err = normalize_angle(target_heading_rad - theta_rad)

                # ถ้ามุมต่างเยอะมาก (> 30°) ให้หมุนตัวอยู่กับที่ก่อน
                if abs(heading_err) > math.radians(30.0):
                    linear_v = 0.0
                    angular_w = math.copysign(
                        min(abs(heading_err) * self.kp_angle, self.max_angular_speed),
                        heading_err,
                    )
                else:
                    # เดินหน้าพร้อมเลี้ยวโค้งปรับมุม
                    linear_v = min(
                        max(distance_err * self.kp_dist, self.min_linear_speed),
                        self.max_linear_speed,
                    )
                    angular_w = math.copysign(
                        min(abs(heading_err) * self.kp_angle, self.max_angular_speed),
                        heading_err,
                    )

                self._motion.drive_continuous(linear_v, angular_w, wheel_base=WHEEL_BASE)

            else:
                # 5. ถึงพิกัดแล้ว จัดการมุมหัวสุดท้าย (Orientation Phase)
                if target_theta_deg is not None:
                    target_theta_rad = math.radians(target_theta_deg)
                    final_heading_err = normalize_angle(target_theta_rad - theta_rad)

                    if abs(final_heading_err) > math.radians(HEADING_TOLERANCE_DEG):
                        angular_w = math.copysign(
                            min(abs(final_heading_err) * self.kp_angle, self.max_angular_speed),
                            final_heading_err,
                        )
                        self._motion.drive_continuous(0.0, angular_w, wheel_base=WHEEL_BASE)
                        time.sleep(self.control_period)
                        continue

                # ถึงเป้าหมายและมุมเรียบร้อย
                logger.info(f"[Nav] Arrived successfully at ({target_x:.2f}, {target_y:.2f})")
                self._motion.stop_continuous()
                self._active = False
                return True

            time.sleep(self.control_period)

        self._motion.stop_continuous()
        return False

    def cancel(self) -> None:
        """ยกเลิกการนำทางและหยุดมอเตอร์"""
        self._active = False
        self._motion.stop_continuous()

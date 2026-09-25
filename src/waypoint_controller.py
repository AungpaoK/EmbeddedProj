#!/usr/bin/env python3
"""Closed-loop controller for the restaurant's fixed delivery route.

The physical route is a small graph rather than a free-space planner:

    kitchen -> junction -> table 1 / table 2

Each mission records the current odometry heading as ``H``. Straight legs
hold one of the planned headings relative to ``H`` and turns close the loop on
odometry before the next leg begins. This is the control strategy used by the
proven real-robot scenario runner, exposed here for the POS-driven FSM.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Callable, Optional, Tuple

from config import ARRIVAL_TOLERANCE_M, JUNCTION_X, TABLE1_Y, TABLE2_Y

logger = logging.getLogger(__name__)


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to ``[-pi, +pi]``."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class WaypointController:
    """Drive the fixed kitchen/junction/table route using odometry feedback."""

    def __init__(
        self,
        motion,
        safety_guard,
        pose_provider: Callable[[], Tuple[float, float, float]],
        *,
        ready_provider: Callable[[], bool] | None = None,
        pose_fresh_provider: Callable[[], bool] | None = None,
        linear_speed: float = 0.22,
        max_angular_speed: float = 0.75,
        control_rate_hz: float = 20.0,
        preflight_timeout_s: float = 12.0,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._motion = motion
        self._safety = safety_guard
        self._get_pose = pose_provider
        self._is_ready = ready_provider or (lambda: True)
        self._is_pose_fresh = pose_fresh_provider or (lambda: True)
        self._clock = monotonic
        self._sleep = sleep

        self.linear_speed = linear_speed
        self.max_angular_speed = max_angular_speed
        self.control_period = 1.0 / control_rate_hz
        self.preflight_timeout_s = preflight_timeout_s

        self._active = False
        self._start_heading: float | None = None
        self._location = "home"
        self.last_error: str | None = None

    def begin_mission(self) -> bool:
        """Wait for healthy motion feedback and capture the mission heading."""
        self.last_error = None
        deadline = self._clock() + self.preflight_timeout_s
        while self._clock() < deadline:
            if self._is_ready() and self._is_pose_fresh():
                _x, _y, heading = self._get_pose()
                self._start_heading = normalize_angle(heading)
                self._location = "home"
                self._active = True
                logger.info(
                    "[Route] Mission frame captured at heading %.1f degrees.",
                    math.degrees(self._start_heading),
                )
                return True
            self._motion.stop_continuous()
            self._sleep(0.1)

        return self._fail("ไม่พบ /arduino/ready และ /odom ที่พร้อมใช้งานภายใน 12 วินาที")

    def go_to_table(self, table_id: int) -> bool:
        """Move from the current logical route location to a selected table."""
        if table_id not in (1, 2):
            return self._fail(f"ไม่รู้จักโต๊ะ {table_id}")
        if self._start_heading is None:
            return self._fail("ยังไม่ได้เริ่ม mission frame")

        destination = f"table_{table_id}"
        if self._location == destination:
            logger.info("[Route] Already at table %d; no movement required.", table_id)
            return True

        side = 1.0 if table_id == 1 else -1.0
        target_heading = normalize_angle(self._start_heading + side * math.pi / 2.0)
        target_distance = TABLE1_Y if table_id == 1 else TABLE2_Y

        if self._location == "home":
            if not self.drive_forward(JUNCTION_X, self._start_heading):
                return False
            if not self.turn_to_heading(target_heading):
                return False
            if not self.drive_forward(target_distance, target_heading):
                return False
        elif self._location in {"table_1", "table_2"}:
            previous_table = 1 if self._location == "table_1" else 2
            previous_distance = TABLE1_Y if previous_table == 1 else TABLE2_Y
            if not self.turn_to_heading(target_heading):
                return False
            if not self.drive_forward(previous_distance + target_distance, target_heading):
                return False
        else:
            return self._fail(f"ตำแหน่งเส้นทางไม่ถูกต้อง: {self._location}")

        self._location = destination
        return True

    def return_home(self) -> bool:
        """Return through the junction and finish facing out toward the aisle."""
        if self._start_heading is None:
            return self._fail("ยังไม่ได้เริ่ม mission frame")

        if self._location == "home":
            return self.turn_to_heading(self._start_heading)
        if self._location not in {"table_1", "table_2"}:
            return self._fail(f"ตำแหน่งเส้นทางไม่ถูกต้อง: {self._location}")

        table_id = 1 if self._location == "table_1" else 2
        side = 1.0 if table_id == 1 else -1.0
        table_distance = TABLE1_Y if table_id == 1 else TABLE2_Y
        heading_to_junction = normalize_angle(self._start_heading - side * math.pi / 2.0)
        heading_to_kitchen = normalize_angle(self._start_heading + math.pi)

        if not self.turn_to_heading(heading_to_junction):
            return False
        if not self.drive_forward(table_distance, heading_to_junction):
            return False
        if not self.turn_to_heading(heading_to_kitchen):
            return False
        if not self.drive_forward(JUNCTION_X, heading_to_kitchen):
            return False
        if not self.turn_to_heading(self._start_heading):
            return False

        self._location = "home"
        self._active = False
        return True

    def drive_forward(
        self,
        distance: float,
        target_heading: float,
        *,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """Drive a measured distance while correcting yaw drift."""
        if distance <= ARRIVAL_TOLERANCE_M:
            return True
        if not self._feedback_healthy():
            return self._fail("Arduino หรือ odometry ไม่พร้อมก่อนเริ่มเดินหน้า")

        target_heading = normalize_angle(target_heading)
        last_x, last_y, _heading = self._get_pose()
        traveled = 0.0
        timeout_s = timeout_s or (distance / max(self.linear_speed, 0.05)) * 2.5 + 5.0
        deadline = self._clock() + timeout_s
        last_progress_time = self._clock()
        last_progress_distance = 0.0

        logger.info(
            "[Route] Drive %.2fm at heading %.1f degrees.",
            distance,
            math.degrees(target_heading),
        )

        while self._active and traveled < distance:
            if not self._feedback_healthy():
                return self._fail("Arduino หรือ odometry ขาดการตอบสนองระหว่างเดินหน้า")
            if self._clock() > deadline:
                return self._fail(f"เดินหน้าไม่ครบระยะ {traveled:.2f}/{distance:.2f} เมตร")

            if self._safety.is_obstacle_detected:
                paused_at = self._clock()
                self._motion.stop_continuous()
                self._sleep(self.control_period)
                paused_for = self._clock() - paused_at
                deadline += paused_for
                last_progress_time += paused_for
                continue

            x, y, heading = self._get_pose()
            heading_error = normalize_angle(target_heading - heading)
            correction = 0.0
            if abs(heading_error) > math.radians(1.0):
                correction = max(-0.5, min(0.5, 1.8 * heading_error))
                if abs(correction) < 0.12:
                    correction = math.copysign(0.12, correction)

            if not self._motion.drive_continuous(self.linear_speed, correction):
                return self._fail("ส่งคำสั่งความเร็วเดินหน้าไม่สำเร็จ")

            self._sleep(self.control_period)
            x, y, _heading = self._get_pose()
            traveled += math.hypot(x - last_x, y - last_y)
            last_x, last_y = x, y

            now = self._clock()
            if traveled - last_progress_distance >= 0.01:
                last_progress_distance = traveled
                last_progress_time = now
            elif now - last_progress_time > 4.0:
                return self._fail("ไม่พบความคืบหน้าจาก odometry ระหว่างเดินหน้าเกิน 4 วินาที")

        self._motion.stop_continuous()
        if traveled >= max(0.0, distance - ARRIVAL_TOLERANCE_M):
            logger.info("[Route] Straight leg completed: %.2fm.", traveled)
            return True
        return self._fail(f"เดินหน้าไม่ครบระยะ {traveled:.2f}/{distance:.2f} เมตร")

    def turn_to_heading(
        self,
        target_heading: float,
        *,
        tolerance_degrees: float = 2.5,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """Turn in place to an absolute mission-relative odometry heading."""
        if not self._feedback_healthy():
            return self._fail("Arduino หรือ odometry ไม่พร้อมก่อนเริ่มหมุน")

        target_heading = normalize_angle(target_heading)
        _x, _y, heading = self._get_pose()
        initial_error = normalize_angle(target_heading - heading)
        tolerance = math.radians(tolerance_degrees)
        if abs(initial_error) <= tolerance:
            self._motion.stop_continuous()
            return True

        timeout_s = timeout_s or (abs(initial_error) / max(self.max_angular_speed, 0.1)) * 3.0 + 6.0
        deadline = self._clock() + timeout_s
        last_progress_time = self._clock()
        best_remaining = abs(initial_error)
        slowdown_range = math.radians(45.0)
        minimum_turn_speed = min(self.max_angular_speed, 0.60)

        logger.info(
            "[Route] Turn to heading %.1f degrees (error %+.1f degrees).",
            math.degrees(target_heading),
            math.degrees(initial_error),
        )

        while self._active:
            if not self._feedback_healthy():
                return self._fail("Arduino หรือ odometry ขาดการตอบสนองระหว่างหมุน")
            if self._clock() > deadline:
                return self._fail("หมุนไม่ถึง heading เป้าหมายภายในเวลาที่กำหนด")

            if self._safety.is_obstacle_detected:
                paused_at = self._clock()
                self._motion.stop_continuous()
                self._sleep(self.control_period)
                paused_for = self._clock() - paused_at
                deadline += paused_for
                last_progress_time += paused_for
                continue

            _x, _y, heading = self._get_pose()
            error = normalize_angle(target_heading - heading)
            remaining = abs(error)

            if remaining <= tolerance:
                self._motion.stop_continuous()
                self._sleep(0.15)
                if not self._feedback_healthy():
                    return self._fail("odometry ขาดการตอบสนองขณะตรวจมุมหลังหยุด")
                _x, _y, settled_heading = self._get_pose()
                settled_error = normalize_angle(target_heading - settled_heading)
                if abs(settled_error) <= tolerance:
                    logger.info(
                        "[Route] Turn completed at %.1f degrees.",
                        math.degrees(settled_heading),
                    )
                    return True
                best_remaining = abs(settled_error)
                last_progress_time = self._clock()
                continue

            now = self._clock()
            if remaining < best_remaining - math.radians(1.5):
                best_remaining = remaining
                last_progress_time = now
            elif now - last_progress_time > 4.0:
                return self._fail("ไม่พบการเปลี่ยนมุมจาก odometry เกิน 4 วินาที")

            if remaining >= slowdown_range:
                turn_speed = self.max_angular_speed
            else:
                progress = max(
                    0.0,
                    min(1.0, (remaining - tolerance) / (slowdown_range - tolerance)),
                )
                turn_speed = minimum_turn_speed + (
                    self.max_angular_speed - minimum_turn_speed
                ) * progress

            angular_speed = math.copysign(turn_speed, error)
            if not self._motion.drive_continuous(0.0, angular_speed):
                return self._fail("ส่งคำสั่งความเร็วหมุนไม่สำเร็จ")
            self._sleep(self.control_period)

        return self._fail("ภารกิจนำทางถูกยกเลิก")

    def cancel(self) -> None:
        self._active = False
        self._motion.stop_continuous()

    def _feedback_healthy(self) -> bool:
        return bool(self._is_ready() and self._is_pose_fresh())

    def _fail(self, message: str) -> bool:
        self.last_error = message
        logger.error("[Route] %s", message)
        self._motion.stop_continuous()
        return False

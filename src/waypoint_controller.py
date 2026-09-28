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
import threading
import time
from typing import Callable, Optional, Tuple

from config import (
    ARRIVAL_TOLERANCE_M,
    CROSS_TRACK_CONTROL_ENABLED,
    CROSS_TRACK_GAIN,
    HEADING_GAIN,
    JUNCTION_X,
    MAX_CROSS_TRACK_CORRECTION,
    MIN_LINEAR_SPEED,
    TELEMETRY_INTERVAL_S,
    TABLE1_Y,
    TABLE2_Y,
    TABLE_STOP_TOLERANCE_M,
    WAYPOINT_HEADING_TOLERANCE_DEG,
    WAYPOINT_POSITION_TOLERANCE_M,
)
from turn_indicator import TURN_LEFT, TURN_OFF, TURN_RIGHT

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
        cancel_event: threading.Event | None = None,
        obstacle_handler: Callable[[bool], None] | None = None,
        cross_track_enabled: bool | None = None,
        cross_track_gain: float | None = None,
        heading_gain: float | None = None,
        max_cross_track_correction: float | None = None,
        min_linear_speed: float | None = None,
        waypoint_position_tolerance_m: float | None = None,
        waypoint_heading_tolerance_degrees: float | None = None,
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
        self._cancel_event = cancel_event or threading.Event()
        self._obstacle_handler = obstacle_handler
        self._obstacle_reported = False

        self.cross_track_enabled = (
            CROSS_TRACK_CONTROL_ENABLED
            if cross_track_enabled is None
            else bool(cross_track_enabled)
        )
        self.cross_track_gain = (
            CROSS_TRACK_GAIN if cross_track_gain is None else float(cross_track_gain)
        )
        self.heading_gain = HEADING_GAIN if heading_gain is None else float(heading_gain)
        self.max_cross_track_correction = (
            MAX_CROSS_TRACK_CORRECTION
            if max_cross_track_correction is None
            else float(max_cross_track_correction)
        )
        self.min_linear_speed = (
            MIN_LINEAR_SPEED if min_linear_speed is None else float(min_linear_speed)
        )
        self.waypoint_position_tolerance_m = (
            WAYPOINT_POSITION_TOLERANCE_M
            if waypoint_position_tolerance_m is None
            else float(waypoint_position_tolerance_m)
        )
        self.waypoint_heading_tolerance_degrees = (
            WAYPOINT_HEADING_TOLERANCE_DEG
            if waypoint_heading_tolerance_degrees is None
            else float(waypoint_heading_tolerance_degrees)
        )

        self._active = False
        self._start_heading: float | None = None
        self._mission_origin: tuple[float, float] | None = None
        self._location = "home"
        self.last_error: str | None = None
        self._last_telemetry_time = float("-inf")

    def begin_mission(self) -> bool:
        """Wait for healthy motion feedback and capture the mission heading."""
        self.last_error = None
        deadline = self._clock() + self.preflight_timeout_s
        while self._clock() < deadline:
            if self._cancel_event.is_set():
                self._motion.stop_continuous()
                return False
            if self._is_ready() and self._is_pose_fresh():
                x, y, heading = self._get_pose()
                self._start_heading = normalize_angle(heading)
                self._mission_origin = (x, y)
                self._location = "home"
                self._active = True
                logger.info(
                    "[Route] Mission frame captured at origin=(%.2f, %.2f), heading %.1f degrees.",
                    x,
                    y,
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
            if not self.drive_to_waypoint(
                JUNCTION_X,
                0.0,
                0.0,
                line_start=(0.0, 0.0),
            ):
                return False
            if not self.turn_to_heading(target_heading):
                return False
            if not self.drive_to_waypoint(
                JUNCTION_X,
                side * target_distance,
                side * math.pi / 2.0,
                line_start=(JUNCTION_X, 0.0),
                table_id=table_id,
            ):
                return False
        elif self._location in {"table_1", "table_2"}:
            previous_table = 1 if self._location == "table_1" else 2
            previous_distance = TABLE1_Y if previous_table == 1 else TABLE2_Y
            if not self.turn_to_heading(target_heading):
                return False
            previous_side = 1.0 if previous_table == 1 else -1.0
            if not self.drive_to_waypoint(
                JUNCTION_X,
                side * target_distance,
                side * math.pi / 2.0,
                line_start=(JUNCTION_X, previous_side * previous_distance),
                table_id=table_id,
            ):
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
        if not self.drive_to_waypoint(
            JUNCTION_X,
            0.0,
            -side * math.pi / 2.0,
            line_start=(JUNCTION_X, side * table_distance),
        ):
            return False
        if not self.turn_to_heading(heading_to_kitchen):
            return False
        if not self.drive_to_waypoint(
            0.0,
            0.0,
            math.pi,
            line_start=(JUNCTION_X, 0.0),
        ):
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
        table_id: int | None = None,
    ) -> bool:
        """Compatibility wrapper that drives one segment from the current pose.

        Route missions use :meth:`drive_to_waypoint`, which knows the nominal
        segment endpoints.  Keeping this wrapper preserves the public API used
        by older callers and unit tests while changing arrival detection to a
        position-based check.
        """
        if self._start_heading is None or self._mission_origin is None:
            return self._fail("ยังไม่ได้เริ่ม mission frame")

        x, y, _heading = self._get_pose()
        current_x, current_y, _current_heading = self._to_mission_pose(x, y, _heading)
        local_heading = normalize_angle(target_heading - self._start_heading)
        target_x = current_x + distance * math.cos(local_heading)
        target_y = current_y + distance * math.sin(local_heading)
        return self._drive_to_waypoint_local(
            target_x,
            target_y,
            local_heading,
            line_start=(current_x, current_y),
            timeout_s=timeout_s,
            table_id=table_id,
        )

    def drive_to_waypoint(
        self,
        target_x: float,
        target_y: float,
        target_heading: float,
        *,
        line_start: tuple[float, float] = (0.0, 0.0),
        timeout_s: Optional[float] = None,
        table_id: int | None = None,
    ) -> bool:
        """Drive to a mission-frame waypoint along a nominal line segment.

        ``target_x``, ``target_y``, ``target_heading`` and ``line_start`` are
        expressed in the mission frame captured by :meth:`begin_mission`:
        home is ``(0, 0)`` and the initial heading is zero.  The controller
        transforms this geometry into odometry coordinates and combines
        heading feedback with signed cross-track feedback.
        """
        if self._start_heading is None or self._mission_origin is None:
            return self._fail("ยังไม่ได้เริ่ม mission frame")
        if table_id is not None and table_id not in (1, 2):
            return self._fail(f"ไม่รู้จักโต๊ะ {table_id}")

        return self._drive_to_waypoint_local(
            float(target_x),
            float(target_y),
            normalize_angle(float(target_heading)),
            line_start=(float(line_start[0]), float(line_start[1])),
            timeout_s=timeout_s,
            table_id=table_id,
        )

    def _drive_to_waypoint_local(
        self,
        target_x: float,
        target_y: float,
        target_heading: float,
        *,
        line_start: tuple[float, float],
        timeout_s: Optional[float],
        table_id: int | None,
    ) -> bool:
        """Implementation for a fixed-route segment in mission coordinates."""
        if table_id is not None and table_id not in (1, 2):
            return self._fail(f"ไม่รู้จักโต๊ะ {table_id}")

        arrival_tolerance = (
            TABLE_STOP_TOLERANCE_M if table_id is not None else ARRIVAL_TOLERANCE_M
        )
        arrival_tolerance = max(
            arrival_tolerance,
            self.waypoint_position_tolerance_m,
        )
        segment_distance = math.hypot(target_x - line_start[0], target_y - line_start[1])
        if not self._feedback_healthy():
            return self._fail("Arduino หรือ odometry ไม่พร้อมก่อนเริ่มเดินหน้า")

        target_heading = normalize_angle(target_heading)
        line_heading = normalize_angle(math.atan2(target_y - line_start[1], target_x - line_start[0]))
        # The route heading is authoritative for the final pose, while the
        # geometric line supplies the cross-track reference.  They should
        # normally agree; using the explicit route heading keeps heading
        # tolerances meaningful even when a route is edited later.
        if segment_distance > 0.0 and abs(normalize_angle(target_heading - line_heading)) > math.radians(5.0):
            logger.warning(
                "[Route] Segment heading %.1f° differs from line heading %.1f°.",
                math.degrees(target_heading),
                math.degrees(line_heading),
            )

        _, _, target_odom_heading = self._from_mission_pose(
            target_x, target_y, target_heading
        )
        last_x, last_y, _heading = self._get_pose()
        traveled = 0.0
        timeout_s = timeout_s or (segment_distance / max(self.linear_speed, 0.05)) * 2.5 + 5.0
        deadline = self._clock() + timeout_s
        last_progress_time = self._clock()
        last_progress_distance = 0.0

        logger.info(
            "[Route] Drive to waypoint local=(%.2f, %.2f), heading %.1f°, segment %.2fm, cross-track=%s.",
            target_x,
            target_y,
            math.degrees(target_heading),
            segment_distance,
            "on" if self.cross_track_enabled else "off",
        )

        while self._active and not self._cancel_event.is_set():
            if not self._feedback_healthy():
                return self._fail("Arduino หรือ odometry ขาดการตอบสนองระหว่างเดินหน้า")
            if self._clock() > deadline:
                return self._fail(
                    f"เดินหน้าไม่ถึง waypoint: remaining={self._remaining_distance(target_x, target_y):.2f}m"
                )

            x, y, heading = self._get_pose()
            local_x, local_y, local_heading = self._to_mission_pose(x, y, heading)
            remaining = math.hypot(target_x - local_x, target_y - local_y)
            heading_error = normalize_angle(target_heading - local_heading)
            cross_track_error = self._cross_track_error(
                local_x,
                local_y,
                line_start,
                line_heading,
            )

            # The parking pose is the navigation goal. Check it first so a
            # physical table that is expected to remain in front of the robot
            # cannot leave the POS in an obstacle state after arrival.
            if remaining <= arrival_tolerance:
                self._motion.stop_continuous()
                if abs(math.degrees(heading_error)) <= self.waypoint_heading_tolerance_degrees:
                    return self._complete_straight_leg(
                        remaining=remaining,
                        table_id=table_id,
                    )
                if not self.turn_to_heading(
                    target_odom_heading,
                    tolerance_degrees=self.waypoint_heading_tolerance_degrees,
                ):
                    return False
                x, y, heading = self._get_pose()
                local_x, local_y, local_heading = self._to_mission_pose(x, y, heading)
                remaining = math.hypot(target_x - local_x, target_y - local_y)
                heading_error = normalize_angle(target_heading - local_heading)
                if (
                    remaining <= arrival_tolerance
                    and abs(math.degrees(heading_error)) <= self.waypoint_heading_tolerance_degrees
                ):
                    return self._complete_straight_leg(
                        remaining=remaining,
                        table_id=table_id,
                    )

            self._report_obstacle_state()
            # _report_obstacle_state samples the safety guard once and keeps
            # the result edge-triggered. Do not read the sensor a second time
            # in the same cycle; some safety providers update their snapshot
            # on property access.
            if self._obstacle_reported:
                paused_at = self._clock()
                self._motion.stop_continuous()
                self._sleep(self.control_period)
                paused_for = self._clock() - paused_at
                deadline += paused_for
                last_progress_time += paused_for
                continue

            correction = self._compute_drive_correction(heading_error, cross_track_error)
            command_speed = self._compute_drive_speed(heading_error, cross_track_error)

            self._log_navigation_telemetry(
                local_x=local_x,
                local_y=local_y,
                local_heading=local_heading,
                target_x=target_x,
                target_y=target_y,
                heading_error=heading_error,
                cross_track_error=cross_track_error,
                remaining=remaining,
                linear_speed=command_speed,
                angular_speed=correction,
            )

            if not self._motion.drive_continuous(command_speed, correction):
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
        if self._cancel_event.is_set():
            return False
        return self._fail(
            f"ภารกิจนำทางหยุดก่อนถึง waypoint: remaining={self._remaining_distance(target_x, target_y):.2f}m"
        )

    def _to_mission_pose(
        self,
        x: float,
        y: float,
        heading: float,
    ) -> tuple[float, float, float]:
        """Transform an odometry pose into the mission-local route frame."""
        if self._mission_origin is None or self._start_heading is None:
            raise RuntimeError("mission frame is not initialized")
        origin_x, origin_y = self._mission_origin
        dx = x - origin_x
        dy = y - origin_y
        c = math.cos(self._start_heading)
        s = math.sin(self._start_heading)
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
            normalize_angle(heading - self._start_heading),
        )

    def _from_mission_pose(
        self,
        x: float,
        y: float,
        heading: float,
    ) -> tuple[float, float, float]:
        """Transform a mission-local route pose into odometry coordinates."""
        if self._mission_origin is None or self._start_heading is None:
            raise RuntimeError("mission frame is not initialized")
        origin_x, origin_y = self._mission_origin
        c = math.cos(self._start_heading)
        s = math.sin(self._start_heading)
        return (
            origin_x + c * x - s * y,
            origin_y + s * x + c * y,
            normalize_angle(self._start_heading + heading),
        )

    @staticmethod
    def _cross_track_error(
        x: float,
        y: float,
        line_start: tuple[float, float],
        line_heading: float,
    ) -> float:
        """Return signed lateral error; positive means left of the line."""
        dx = x - line_start[0]
        dy = y - line_start[1]
        return -math.sin(line_heading) * dx + math.cos(line_heading) * dy

    def _compute_drive_correction(
        self,
        heading_error: float,
        cross_track_error: float,
    ) -> float:
        """Combine heading and cross-track feedback into angular velocity."""
        correction = 0.0
        if abs(heading_error) > math.radians(1.0):
            correction = max(-0.5, min(0.5, self.heading_gain * heading_error))
            if not self.cross_track_enabled and abs(correction) < 0.12:
                correction = math.copysign(0.12, correction)

        if self.cross_track_enabled and self.cross_track_gain > 0.0:
            cross_correction = max(
                -self.max_cross_track_correction,
                min(
                    self.max_cross_track_correction,
                    -self.cross_track_gain * cross_track_error,
                ),
            )
            correction += cross_correction

        return max(-0.5, min(0.5, correction))

    def _compute_drive_speed(
        self,
        heading_error: float,
        cross_track_error: float,
    ) -> float:
        """Slow down while recovering a large lateral or angular error."""
        if not self.cross_track_enabled:
            return self.linear_speed
        if (
            abs(cross_track_error) > 0.10
            or abs(heading_error) > math.radians(15.0)
        ):
            return max(self.min_linear_speed, self.linear_speed * 0.60)
        return self.linear_speed

    def _remaining_distance(self, target_x: float, target_y: float) -> float:
        x, y, heading = self._get_pose()
        local_x, local_y, _ = self._to_mission_pose(x, y, heading)
        return math.hypot(target_x - local_x, target_y - local_y)

    def _log_navigation_telemetry(
        self,
        *,
        local_x: float,
        local_y: float,
        local_heading: float,
        target_x: float,
        target_y: float,
        heading_error: float,
        cross_track_error: float,
        remaining: float,
        linear_speed: float,
        angular_speed: float,
    ) -> None:
        now = self._clock()
        if now - self._last_telemetry_time < TELEMETRY_INTERVAL_S:
            return
        self._last_telemetry_time = now
        logger.info(
            "[RouteTelemetry] pose=(%.2f, %.2f, %.1f°) target=(%.2f, %.2f) "
            "remaining=%.2fm cross_track=%+.3fm heading_error=%+.1f° "
            "cmd=(v=%.2f,w=%+.2f) obstacle=%s",
            local_x,
            local_y,
            math.degrees(local_heading),
            target_x,
            target_y,
            remaining,
            cross_track_error,
            math.degrees(heading_error),
            linear_speed,
            angular_speed,
            self._obstacle_reported,
        )

    def _complete_straight_leg(
        self,
        *,
        remaining: float,
        table_id: int | None,
    ) -> bool:
        """Stop at a completed leg and clear stale obstacle UI state."""
        self._motion.stop_continuous()
        if table_id is not None:
            # A table is expected to remain visible in the front LiDAR cone
            # while the robot waits for pickup. It is the reached destination,
            # not a currently actionable blockage for the POS display.
            if self._obstacle_reported:
                self._set_obstacle_reported(False)
            logger.info(
                "[Route] Reached parking pose for table %d: remaining %.2fm.",
                table_id,
                remaining,
            )
        else:
            logger.info("[Route] Waypoint reached: remaining %.2fm.", remaining)
        return True

    def turn_to_heading(
        self,
        target_heading: float,
        *,
        tolerance_degrees: float = 2.5,
        timeout_s: Optional[float] = None,
    ) -> bool:
        target_heading = normalize_angle(target_heading)
        _x, _y, heading = self._get_pose()
        initial_error = normalize_angle(target_heading - heading)
        tolerance = math.radians(tolerance_degrees)
        if abs(initial_error) <= tolerance:
            self._motion.set_turn_intent(TURN_OFF)
            self._motion.stop_continuous()
            return True

        if not self._feedback_healthy():
            return self._fail("Arduino หรือ odometry ไม่พร้อมก่อนเริ่มหมุน")

        direction = TURN_LEFT if initial_error > 0.0 else TURN_RIGHT
        try:
            self._motion.set_turn_intent(direction)
            return self._turn_to_heading_active(
                target_heading,
                tolerance_degrees=tolerance_degrees,
                timeout_s=timeout_s,
            )
        finally:
            self._motion.set_turn_intent(TURN_OFF)

    def _turn_to_heading_active(
        self,
        target_heading: float,
        *,
        tolerance_degrees: float,
        timeout_s: Optional[float],
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

        while self._active and not self._cancel_event.is_set():
            if not self._feedback_healthy():
                return self._fail("Arduino หรือ odometry ขาดการตอบสนองระหว่างหมุน")
            if self._clock() > deadline:
                return self._fail("หมุนไม่ถึง heading เป้าหมายภายในเวลาที่กำหนด")

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

        self._motion.stop_continuous()
        if self._cancel_event.is_set():
            return False
        return self._fail("ภารกิจนำทางถูกยกเลิก")

    def _report_obstacle_state(self) -> None:
        detected = bool(self._safety.is_obstacle_detected)
        if detected == self._obstacle_reported:
            return
        self._set_obstacle_reported(detected)

    def _set_obstacle_reported(self, detected: bool) -> None:
        """Update the edge-triggered obstacle state exposed to the POS."""
        self._obstacle_reported = bool(detected)
        if self._obstacle_handler is not None:
            self._obstacle_handler(self._obstacle_reported)

    def cancel(self) -> None:
        if self._obstacle_handler is not None:
            self._obstacle_handler(False)
        self._obstacle_reported = False
        self._cancel_event.set()
        self._active = False
        self._motion.stop_continuous()
        self._motion.set_turn_intent(TURN_OFF)
        self._motion.set_delivery_mission_active(False)

    def _feedback_healthy(self) -> bool:
        return bool(self._is_ready() and self._is_pose_fresh())

    def _fail(self, message: str) -> bool:
        self.last_error = message
        logger.error("[Route] %s", message)
        self._motion.stop_continuous()
        return False

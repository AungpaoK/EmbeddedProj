#!/usr/bin/env python3
"""
odometry.py — Differential Drive Odometry Calculator
=====================================================
อ่านค่า Encoder Ticks จาก Arduino #1 ผ่าน Serial
แล้วคำนวณตำแหน่ง (x, y, theta) แบบ Dead Reckoning

Protocol รับ: "ENCODER:<left_ticks>,<right_ticks>\\n"
"""

import math
import threading
import time
try:
    import serial
except ImportError:
    serial = None
import logging

from config import (
    WHEEL_RADIUS,
    WHEEL_BASE,
    TICKS_PER_REV,
    METERS_PER_TICK,
)

logger = logging.getLogger(__name__)


class Odometry:
    """
    คำนวณ Odometry (x, y, theta) จาก Encoder Ticks แบบ Real-time
    รันใน Background Thread แยกจาก FSM
    """

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._lock = threading.Lock()

        # --- Pose State ---
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0  # radians

        # --- Encoder Reference ---
        self._last_left_ticks: int | None = None
        self._last_right_ticks: int | None = None

        # --- Background Reader Thread ---
        self._running = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """เริ่ม background thread อ่าน Encoder"""
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("[Odometry] Background reader started.")

    def stop(self) -> None:
        """หยุด background thread"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("[Odometry] Background reader stopped.")

    def reset(self) -> None:
        """รีเซ็ตตำแหน่งกลับเป็น (0, 0, 0°) — เรียกเมื่อหุ่นกลับถึง Station"""
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self._last_left_ticks = None
            self._last_right_ticks = None
        logger.info("[Odometry] Pose reset to (0, 0, 0°).")

    @property
    def pose(self) -> tuple[float, float, float]:
        """คืนค่า (x, y, theta) thread-safe"""
        with self._lock:
            return self.x, self.y, self.theta

    @property
    def theta_deg(self) -> float:
        """คืนค่ามุมหันหน้าเป็นองศา thread-safe"""
        with self._lock:
            return math.degrees(self.theta)

    def distance_to(self, target_x: float, target_y: float) -> float:
        """ระยะทางยูคลิดจากตำแหน่งปัจจุบันถึง target"""
        x, y, _ = self.pose
        return math.hypot(target_x - x, target_y - y)

    def angle_to_deg(self, target_x: float, target_y: float) -> float:
        """
        คำนวณมุมต่างระหว่าง heading ปัจจุบันกับทิศที่ต้องการ (องศา)
        บวก = ต้องเลี้ยวซ้าย (CCW), ลบ = ต้องเลี้ยวขวา (CW)
        """
        x, y, theta = self.pose
        desired = math.atan2(target_y - y, target_x - x)
        diff = desired - theta
        # Normalize to [-π, +π]
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return math.degrees(diff)

    # ------------------------------------------------------------------
    # Internal: Background Serial Reader
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            try:
                if self._ser.in_waiting == 0:
                    time.sleep(0.005)
                    continue

                raw = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw.startswith("ENCODER:"):
                    continue

                payload = raw[len("ENCODER:"):]
                parts = payload.split(",")
                if len(parts) != 2:
                    continue

                left_ticks = int(parts[0])
                right_ticks = int(parts[1])
                self._integrate(left_ticks, right_ticks)

            except (ValueError, UnicodeDecodeError):
                continue
            except serial.SerialException as e:
                logger.error(f"[Odometry] Serial error: {e}")
                break

    def _integrate(self, left_ticks: int, right_ticks: int) -> None:
        """คำนวณและอัปเดต Odometry ตามสมการ Dead Reckoning"""
        with self._lock:
            if self._last_left_ticks is None:
                self._last_left_ticks = left_ticks
                self._last_right_ticks = right_ticks
                return

            d_left_ticks = left_ticks - self._last_left_ticks
            d_right_ticks = right_ticks - self._last_right_ticks

            self._last_left_ticks = left_ticks
            self._last_right_ticks = right_ticks

            # Convert Ticks → Meters
            d_left = d_left_ticks * METERS_PER_TICK
            d_right = d_right_ticks * METERS_PER_TICK

            # Differential Kinematics
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / WHEEL_BASE

            # Integrate Pose (Midpoint method)
            if d_center != 0:
                self.x += d_center * math.cos(self.theta + d_theta / 2.0)
                self.y += d_center * math.sin(self.theta + d_theta / 2.0)
            self.theta += d_theta

            # Normalize theta to [-π, +π]
            while self.theta > math.pi:
                self.theta -= 2.0 * math.pi
            while self.theta < -math.pi:
                self.theta += 2.0 * math.pi

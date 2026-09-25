#!/usr/bin/env python3
"""
motion_client.py — Motion Command Client (Pi → Arduino #1)
==========================================================
ส่งคำสั่งการเคลื่อนที่ผ่าน Serial ไปยัง Arduino #1 (Motion Controller)
และรอรับสถานะผลลัพธ์กลับมาแบบ Blocking (พร้อม Timeout)

Serial Protocol (Arduino #1):
  ส่งออก (Pi → Arduino):
    FORWARD:<distance_m>\\n   — วิ่งตรงระยะ distance_m เมตร
    TURN:<degrees>\\n          — หมุน degrees องศา (+ = ซ้าย/CCW, - = ขวา/CW)
    INDICATOR:LEFT|RIGHT|OFF — เลือกไฟเลี้ยวจากสถานะบน Pi
    STOP\\n                    — หยุดฉุกเฉิน

  รับกลับ (Arduino → Pi):
    STATUS:DONE\\n             — ทำคำสั่งสำเร็จ
    STATUS:ERROR\\n            — เกิดข้อผิดพลาด
    ENCODER:<L>,<R>\\n         — Encoder Ticks (สำหรับ Odometry, อ่านโดย odometry.py)
"""

try:
    import serial
except ImportError:
    serial = None
import time
import logging

from config import (
    MOTION_COMMAND_TIMEOUT_S,
    TURN_INDICATOR_OFF_THRESHOLD,
    TURN_INDICATOR_ON_THRESHOLD,
    TURN_INDICATOR_SETTLE_SECONDS,
    WHEEL_BASE,
)
from turn_indicator import TurnIndicatorController

logger = logging.getLogger(__name__)


class MotionClient:
    """
    Wrapper ส่งคำสั่ง Motion ไปยัง Arduino #1
    ทุกคำสั่งจะ block จน Arduino ตอบ STATUS:DONE หรือ Timeout
    """

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._turn_indicator = "OFF"
        self._turn_indicator_controller = TurnIndicatorController(
            turn_on_threshold=TURN_INDICATOR_ON_THRESHOLD,
            turn_off_threshold=TURN_INDICATOR_OFF_THRESHOLD,
            settle_seconds=TURN_INDICATOR_SETTLE_SECONDS,
        )

    # ------------------------------------------------------------------
    # Public Motion Commands
    # ------------------------------------------------------------------

    def forward(self, distance_m: float) -> bool:
        """
        สั่งวิ่งตรงระยะ distance_m เมตร
        คืน True ถ้าสำเร็จ, False ถ้า Timeout หรือ Error
        """
        self._set_turn_indicator("OFF")
        cmd = f"FORWARD:{distance_m:.3f}\n"
        logger.info(f"[Motion] → {cmd.strip()}")
        return self._send_and_wait(cmd)

    def turn(self, degrees: float) -> bool:
        """
        สั่งหมุน degrees องศา
        + = เลี้ยวซ้าย (CCW), - = เลี้ยวขวา (CW)
        คืน True ถ้าสำเร็จ
        """
        # Reverse the previous left/right mapping to match the robot's actual
        # travel direction as seen from behind.
        signal = "RIGHT" if degrees > 0 else "LEFT" if degrees < 0 else "OFF"
        self._set_turn_indicator(signal)
        cmd = f"TURN:{degrees:.1f}\n"
        logger.info(f"[Motion] → {cmd.strip()}")
        try:
            return self._send_and_wait(cmd)
        finally:
            self._set_turn_indicator("OFF")

    def stop(self) -> None:
        """ส่งคำสั่งหยุดฉุกเฉิน (ไม่รอ STATUS:DONE)"""
        cmd = "STOP\n"
        logger.warning(f"[Motion] → STOP (emergency)")
        self._set_turn_indicator("OFF")
        try:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
        except serial.SerialException as e:
            logger.error(f"[Motion] Failed to send STOP: {e}")

    def _set_turn_indicator(self, signal: str) -> None:
        self._turn_indicator_controller.force(signal)
        if signal == self._turn_indicator:
            return
        try:
            self._ser.write(f"INDICATOR:{signal}\n".encode("utf-8"))
            self._ser.flush()
            self._turn_indicator = signal
        except serial.SerialException as e:
            logger.error(f"[Motion] Failed to set turn indicator: {e}")

    # ------------------------------------------------------------------
    # Continuous Velocity Commands (ROS 2 cmd_vel Streaming)
    # ------------------------------------------------------------------

    def set_wheel_velocities(self, v_left: float, v_right: float) -> bool:
        """
        ส่งคำสั่งความเร็วล้อซ้ายและขวา (m/s) แบบ Continuous (Non-blocking)
        ตัวอย่าง: V:0.250,0.250
        """
        cmd = f"V:{v_left:.3f},{v_right:.3f}\n"
        try:
            signal = self._turn_indicator_controller.update(
                (v_right - v_left) / WHEEL_BASE,
                time.monotonic(),
            )
            if signal is not None:
                self._set_turn_indicator(signal)
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
            return True
        except serial.SerialException as e:
            logger.error(f"[Motion] Failed to send velocity: {e}")
            return False

    def drive_continuous(self, linear_v: float, angular_w: float, wheel_base: float = 0.343) -> bool:
        """
        แปลง (linear_v m/s, angular_w rad/s) เป็นความเร็วล้อ Differential Drive
        v_left  = linear_v - (angular_w * wheel_base / 2.0)
        v_right = linear_v + (angular_w * wheel_base / 2.0)
        """
        v_left  = linear_v - (angular_w * wheel_base / 2.0)
        v_right = linear_v + (angular_w * wheel_base / 2.0)
        return self.set_wheel_velocities(v_left, v_right)

    def stop_continuous(self) -> bool:
        """หยุดมอเตอร์ในโหมด Continuous ด้วยการส่ง V:0.000,0.000"""
        return self.set_wheel_velocities(0.0, 0.0)

    # ------------------------------------------------------------------
    # Composite Maneuvers (ประกอบจากคำสั่งพื้นฐาน)
    # ------------------------------------------------------------------

    def u_turn(self) -> bool:
        """หมุนตัว 180° in-place (U-Turn)"""
        logger.info("[Motion] → U-TURN (180°)")
        return self.turn(180.0)

    def navigate_to_junction(self, junction_x: float) -> bool:
        """วิ่งตรงจากจุดปัจจุบันไปถึง Junction (ตามแกน X)"""
        return self.forward(junction_x)

    def navigate_forward(self, distance_m: float) -> bool:
        """วิ่งตรงไปตามระยะ distance_m เมตร"""
        return self.forward(distance_m)

    def turn_left_90(self) -> bool:
        """เลี้ยวซ้าย 90°"""
        return self.turn(+90.0)

    def turn_right_90(self) -> bool:
        """เลี้ยวขวา 90°"""
        return self.turn(-90.0)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send_and_wait(self, cmd: str, timeout: float = MOTION_COMMAND_TIMEOUT_S) -> bool:
        """
        ส่งคำสั่ง cmd และรอรับ STATUS:DONE หรือ STATUS:ERROR
        กรอง ENCODER:... ออก เพราะเป็นของ odometry.py
        คืน True ถ้า DONE, False ถ้า ERROR หรือ Timeout
        """
        try:
            self._ser.reset_input_buffer()
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
        except serial.SerialException as e:
            logger.error(f"[Motion] Serial write error: {e}")
            return False

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if self._ser.in_waiting == 0:
                    time.sleep(0.01)
                    continue

                raw = self._ser.readline().decode("utf-8", errors="ignore").strip()

                # กรอง ENCODER line ออก ปล่อยให้ Odometry thread จัดการ
                if raw.startswith("ENCODER:"):
                    continue

                logger.debug(f"[Motion] ← {raw}")

                if raw == "STATUS:DONE":
                    logger.info(f"[Motion] ✓ DONE")
                    return True

                if raw == "STATUS:ERROR":
                    logger.error(f"[Motion] ✗ ERROR reported by Arduino")
                    return False

            except (UnicodeDecodeError, serial.SerialException) as e:
                logger.warning(f"[Motion] Read error: {e}")
                continue

        logger.error(f"[Motion] Timeout waiting for STATUS:DONE (cmd={cmd.strip()})")
        return False


class RosMotionClient:
    """Motion client that delegates the Arduino serial link to slam_bridge.

    ``slam_bridge.py`` is the sole owner of ``/dev/ttyACM0``.  This adapter
    keeps the existing waypoint-controller interface while sending ROS
    ``/cmd_vel`` messages instead of opening the Arduino serial port.
    """

    def __init__(self, ros_node):
        self._ros_node = ros_node

    def forward(self, distance_m: float) -> bool:
        logger.error(
            "[Motion/ROS] forward() is not supported directly; "
            "use WaypointController.drive_continuous()."
        )
        return False

    def turn(self, degrees: float) -> bool:
        logger.error(
            "[Motion/ROS] turn() is not supported directly; "
            "use WaypointController.drive_continuous()."
        )
        return False

    def drive_continuous(self, linear_v: float, angular_w: float, wheel_base: float = 0.343) -> bool:
        if not self._ros_node.motion_ready:
            return False
        return self._ros_node.publish_cmd_vel(linear_v, angular_w)

    def stop_continuous(self) -> bool:
        return self._ros_node.publish_cmd_vel(0.0, 0.0, allow_when_not_ready=True)

    def stop(self) -> None:
        self.stop_continuous()

    def set_wheel_velocities(self, v_left: float, v_right: float) -> bool:
        linear_v = (v_left + v_right) / 2.0
        angular_w = (v_right - v_left) / 0.343
        return self.drive_continuous(linear_v, angular_w, wheel_base=0.343)

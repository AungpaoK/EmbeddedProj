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
    STOP\\n                    — หยุดฉุกเฉิน

  รับกลับ (Arduino → Pi):
    STATUS:DONE\\n             — ทำคำสั่งสำเร็จ
    STATUS:ERROR\\n            — เกิดข้อผิดพลาด
    ENCODER:<L>,<R>\\n         — Encoder Ticks (สำหรับ Odometry, อ่านโดย odometry.py)
"""

import serial
import time
import logging

from config import MOTION_COMMAND_TIMEOUT_S

logger = logging.getLogger(__name__)


class MotionClient:
    """
    Wrapper ส่งคำสั่ง Motion ไปยัง Arduino #1
    ทุกคำสั่งจะ block จน Arduino ตอบ STATUS:DONE หรือ Timeout
    """

    def __init__(self, ser: serial.Serial):
        self._ser = ser

    # ------------------------------------------------------------------
    # Public Motion Commands
    # ------------------------------------------------------------------

    def forward(self, distance_m: float) -> bool:
        """
        สั่งวิ่งตรงระยะ distance_m เมตร
        คืน True ถ้าสำเร็จ, False ถ้า Timeout หรือ Error
        """
        cmd = f"FORWARD:{distance_m:.3f}\n"
        logger.info(f"[Motion] → {cmd.strip()}")
        return self._send_and_wait(cmd)

    def turn(self, degrees: float) -> bool:
        """
        สั่งหมุน degrees องศา
        + = เลี้ยวซ้าย (CCW), - = เลี้ยวขวา (CW)
        คืน True ถ้าสำเร็จ
        """
        cmd = f"TURN:{degrees:.1f}\n"
        logger.info(f"[Motion] → {cmd.strip()}")
        return self._send_and_wait(cmd)

    def stop(self) -> None:
        """ส่งคำสั่งหยุดฉุกเฉิน (ไม่รอ STATUS:DONE)"""
        cmd = "STOP\n"
        logger.warning(f"[Motion] → STOP (emergency)")
        try:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
        except serial.SerialException as e:
            logger.error(f"[Motion] Failed to send STOP: {e}")

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

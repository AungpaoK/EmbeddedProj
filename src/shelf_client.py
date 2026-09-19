#!/usr/bin/env python3
"""
shelf_client.py — Shelf & UI Client (Pi → Arduino #2)
======================================================
สื่อสารกับ Arduino #2 (Shelf & User Interface Controller)
รับสถานะ IR Sensor บนแต่ละชั้น และสัญญาณจาก Keypad / Manual Override

Serial Protocol (Arduino #2):
  รับกลับ (Arduino → Pi):
    IR:<shelf>,<status>\\n     — IR Sensor (shelf=1|2, status=1=มีของ, 0=ไม่มี)
    KEY:<char>\\n              — Keypad กดตัวอักษร เช่น KEY:1, KEY:B, KEY:#, KEY:*
    OVERRIDE\\n                — Manual Override ถูกกด

  ส่งออก (Pi → Arduino):  [ยังไม่ใช้ในเวอร์ชันนี้ — สงวนไว้สำหรับ LCD Display]
    LCD:<row>,<text>\\n        — แสดงข้อความบน LCD row 0 หรือ 1
"""

try:
    import serial
except ImportError:
    serial = None
import threading
import time
import logging
from collections import deque

logger = logging.getLogger(__name__)


class ShelfClient:
    """
    อ่านสถานะ IR Sensor และ Keypad จาก Arduino #2 ใน Background Thread
    FSM เรียกใช้ .poll_key() / .ir_has_food(shelf) เพื่อดึงข้อมูล
    """

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._lock = threading.Lock()

        # IR Sensor state: shelf 1 และ 2 (True = มีอาหาร)
        self._ir: dict[int, bool] = {1: False, 2: False}

        # Keypad queue
        self._key_queue: deque[str] = deque(maxlen=16)

        # Manual Override flag
        self._override_flag = False

        # Background thread
        self._running = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("[Shelf] Background reader started.")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info("[Shelf] Background reader stopped.")

    # ------------------------------------------------------------------
    # Public API — IR Sensor
    # ------------------------------------------------------------------

    def ir_has_food(self, shelf: int) -> bool:
        """True ถ้า IR ตรวจพบอาหารอยู่บนชั้น shelf (1 หรือ 2)"""
        with self._lock:
            return self._ir.get(shelf, False)

    def wait_for_food_on(self, shelf: int, timeout: float | None = None) -> bool:
        """
        Block จนกว่า IR ตรวจพบอาหารบนชั้น shelf
        คืน True ถ้าพบในเวลา timeout, False ถ้า Timeout
        """
        deadline = (time.monotonic() + timeout) if timeout else None
        while True:
            if self.ir_has_food(shelf):
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.05)

    def wait_for_food_removed(self, shelf: int, timeout: float | None = None) -> bool:
        """
        Block จนกว่า IR ตรวจไม่พบอาหาร (อาหารถูกหยิบออก) หรือ Override
        คืน True ถ้าถูกหยิบออก หรือมี Override, False ถ้า Timeout
        """
        deadline = (time.monotonic() + timeout) if timeout else None
        while True:
            if self.consume_override():
                logger.info(f"[Shelf] Manual Override triggered on shelf {shelf}.")
                return True
            if not self.ir_has_food(shelf):
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Public API — Keypad
    # ------------------------------------------------------------------

    def poll_key(self) -> str | None:
        """ดึง Key ออกจาก Queue (FIFO) คืน None ถ้าไม่มีการกด"""
        with self._lock:
            return self._key_queue.popleft() if self._key_queue else None

    def wait_for_key(self, valid_keys: list[str] | None = None, timeout: float | None = None) -> str | None:
        """
        Block จนกว่าจะมีการกด Key ที่ต้องการ
        valid_keys=None = รับทุกปุ่ม
        คืน Key string หรือ None ถ้า Timeout
        """
        deadline = (time.monotonic() + timeout) if timeout else None
        while True:
            key = self.poll_key()
            if key is not None:
                if valid_keys is None or key in valid_keys:
                    return key
            if deadline and time.monotonic() > deadline:
                return None
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # Public API — Manual Override
    # ------------------------------------------------------------------

    def consume_override(self) -> bool:
        """ตรวจและ Clear Override Flag (consume หมายถึงอ่านครั้งเดียวแล้วล้าง)"""
        with self._lock:
            if self._override_flag:
                self._override_flag = False
                return True
            return False

    # ------------------------------------------------------------------
    # Public API — LCD Display (สำรองไว้สำหรับอนาคต)
    # ------------------------------------------------------------------

    def lcd_print(self, row: int, text: str) -> None:
        """ส่งข้อความไปแสดงบน LCD (row 0 หรือ 1)"""
        cmd = f"LCD:{row},{text[:16]}\n"
        try:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()
        except serial.SerialException as e:
            logger.warning(f"[Shelf] LCD write error: {e}")

    # ------------------------------------------------------------------
    # Internal: Background Reader
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        while self._running:
            try:
                if self._ser.in_waiting == 0:
                    time.sleep(0.005)
                    continue

                raw = self._ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                self._parse_line(raw)

            except (UnicodeDecodeError, ValueError):
                continue
            except serial.SerialException as e:
                logger.error(f"[Shelf] Serial error: {e}")
                break

    def _parse_line(self, raw: str) -> None:
        """แยก prefix แล้วอัปเดต state"""
        with self._lock:
            if raw.startswith("IR:"):
                # Format: IR:<shelf>,<status>
                payload = raw[3:]
                parts = payload.split(",")
                if len(parts) == 2:
                    shelf = int(parts[0])
                    status = bool(int(parts[1]))
                    self._ir[shelf] = status
                    logger.debug(f"[Shelf] IR shelf={shelf} food={status}")

            elif raw.startswith("KEY:"):
                key = raw[4:]
                self._key_queue.append(key)
                logger.info(f"[Shelf] Keypad: '{key}'")

            elif raw == "OVERRIDE":
                self._override_flag = True
                logger.warning("[Shelf] OVERRIDE button pressed!")


class VirtualShelfClient:
    """
    Virtual / Mock Shelf & UI Client
    ใช้เมื่อ Arduino #2 ยังไม่พร้อม เพื่อให้สามารถทดสอบระบบ LiDAR, SLAM และการเคลื่อนที่ได้ทันที
    """

    def __init__(
        self,
        auto_dispatch: bool = True,
        default_shelf: int = 1,
        default_table: int = 1,
    ) -> None:
        self.auto_dispatch = auto_dispatch
        self.default_shelf = default_shelf
        self.default_table = default_table

        self._food_on_shelf: dict[int, bool] = {1: False, 2: False}
        self._key_queue: deque[str] = deque()
        self._override_flag = False

    def start(self) -> None:
        logger.info("[VirtualShelf] Virtual Shelf Client started (Mock Hardware UI).")
        print("\n" + "-" * 55)
        print("  💡 Virtual Shelf Mode (Arduino #2 is Optional)")
        print(f"     Auto-Dispatch: Shelf {self.default_shelf} → Table {self.default_table}")
        print("-" * 55)

    def stop(self) -> None:
        logger.info("[VirtualShelf] Virtual Shelf Client stopped.")

    def lcd_print(self, row: int, text: str) -> None:
        print(f"  📺 [LCD Row {row}] {text}")

    def ir_has_food(self, shelf: int) -> bool:
        return self._food_on_shelf.get(shelf, False)

    def wait_for_food_on(self, shelf: int, timeout: float | None = None) -> bool:
        time.sleep(0.8)
        self._food_on_shelf[shelf] = True
        print(f"  🍽️ [Virtual Sensor] ตรวจพบอาหารวางบนชั้น {shelf} (IR = DETECTED)")
        return True

    def wait_for_food_removed(self, shelf: int, timeout: float | None = None) -> bool:
        print(f"  🍽️ [Virtual Sensor] รอส่งมอบอาหารที่ชั้น {shelf}... (จำลองลูกค้ารับอาหารใน 3 วินาที)")
        time.sleep(3.0)
        self._food_on_shelf[shelf] = False
        print(f"  ✅ [Virtual Sensor] ลูกค้าหยิบอาหารชั้น {shelf} ออกแล้ว (IR = CLEARED)")
        return True

    def poll_key(self) -> str | None:
        if self._key_queue:
            return self._key_queue.popleft()
        return None

    def wait_for_key(
        self,
        valid_keys: list[str] | None = None,
        timeout: float | None = None,
    ) -> str | None:
        if self._key_queue:
            k = self._key_queue.popleft()
            if valid_keys is None or k in valid_keys:
                return k

        if self.auto_dispatch and valid_keys:
            time.sleep(0.6)
            # ลำดับจำลองปุ่มของ FSM:
            if str(self.default_shelf) in valid_keys:
                key = str(self.default_shelf)
            elif "B" in valid_keys:
                key = "B"
            elif str(self.default_table) in valid_keys:
                key = str(self.default_table)
            elif "C" in valid_keys:
                key = "C"
            elif "#" in valid_keys:
                key = "#"
            else:
                key = valid_keys[0]

            print(f"  ⌨️ [Virtual Keypad] จำลองการกดปุ่ม: '{key}'")
            return key

        return None

    def consume_override(self) -> bool:
        if self._override_flag:
            self._override_flag = False
            return True
        return False


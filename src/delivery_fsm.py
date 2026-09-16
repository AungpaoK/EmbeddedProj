#!/usr/bin/env python3
"""
delivery_fsm.py — Main Delivery FSM
=====================================
Implements the full Main Delivery FSM ตาม docs/FSM.md และ docs/scenario.md

States:
    SELECT_FLOOR   → รอพนักงานเลือกชั้น (กด 1/2 + B)
    WAIT_FOR_IR    → รอ IR Sensor ตรวจพบอาหาร
    ENTER_TABLE    → รับหมายเลขโต๊ะจาก Keypad
    SHOW_LIST      → แสดงรายการ แล้วรอ # เพื่อส่ง หรือ A เพื่อเพิ่ม
    RESET_ALL      → ล้างงานทั้งหมด กลับสู่ SELECT_FLOOR
    DELIVERING     → ส่งอาหารไปยังโต๊ะ (เรียก Motion Sequence)
    WAIT_PICKUP    → รอ IR ตรวจไม่พบ (อาหารถูกหยิบ) / Manual Override
    CHECK_REMAIN   → ตรวจว่าเหลือโต๊ะต้องส่งอีกไหม
    RETURN_STATION → กลับ Serve Station
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum, auto

from config import (
    JUNCTION_X,
    TABLE1_Y,
    TABLE2_Y,
    PICKUP_WAIT_TIMEOUT_S,
)
from motion_client import MotionClient
from odometry import Odometry
from shelf_client import ShelfClient

logger = logging.getLogger(__name__)


# ===========================================================
# State Enum
# ===========================================================
class State(Enum):
    SELECT_FLOOR   = auto()
    WAIT_FOR_IR    = auto()
    ENTER_TABLE    = auto()
    SHOW_LIST      = auto()
    RESET_ALL      = auto()
    DELIVERING     = auto()
    WAIT_PICKUP    = auto()
    CHECK_REMAIN   = auto()
    RETURN_STATION = auto()


# ===========================================================
# Delivery Order Data
# ===========================================================
@dataclass
class DeliveryOrder:
    """คำสั่งเสิร์ฟหนึ่งรายการ: ชั้น (shelf) → โต๊ะ (table_id)"""
    shelf: int       # 1 หรือ 2
    table_id: int    # 1 หรือ 2


# ===========================================================
# Main Delivery FSM
# ===========================================================
class DeliveryFSM:
    """
    Main Delivery Finite State Machine
    ควบคุมลำดับการส่งอาหารตาม docs/FSM.md
    """

    def __init__(
        self,
        motion: MotionClient,
        odometry: Odometry,
        shelf: ShelfClient,
    ) -> None:
        self._motion = motion
        self._odom = odometry
        self._shelf = shelf

        self._state = State.SELECT_FLOOR

        # รายการคำสั่งเสิร์ฟ (คิว)
        self._orders: list[DeliveryOrder] = []
        self._current_shelf: int | None = None  # ชั้นที่เลือกอยู่ตอน setup

    # ------------------------------------------------------------------
    # Main Run Loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """วน Loop ตามสถานะ FSM จนกว่าจะ Shutdown"""
        logger.info("[FSM] Delivery FSM started.")
        print("\n" + "=" * 55)
        print("  Food Delivery Robot — Main FSM")
        print("=" * 55)

        while True:
            self._step()

    def _step(self) -> None:
        """ดำเนินการ 1 ขั้นตอนตาม state ปัจจุบัน"""
        logger.debug(f"[FSM] State: {self._state.name}")

        match self._state:
            case State.SELECT_FLOOR:
                self._state_select_floor()
            case State.WAIT_FOR_IR:
                self._state_wait_for_ir()
            case State.ENTER_TABLE:
                self._state_enter_table()
            case State.SHOW_LIST:
                self._state_show_list()
            case State.RESET_ALL:
                self._state_reset_all()
            case State.DELIVERING:
                self._state_delivering()
            case State.WAIT_PICKUP:
                self._state_wait_pickup()
            case State.CHECK_REMAIN:
                self._state_check_remain()
            case State.RETURN_STATION:
                self._state_return_station()

    # ------------------------------------------------------------------
    # State Handlers
    # ------------------------------------------------------------------

    def _state_select_floor(self) -> None:
        """รอพนักงานเลือกชั้น (กด 1 หรือ 2) แล้วกด B ยืนยัน"""
        self._print_status("เลือกชั้น (กด 1 หรือ 2) แล้วกด B ยืนยัน")
        self._shelf.lcd_print(0, "Select shelf:")
        self._shelf.lcd_print(1, "[1] or [2] + B")

        while True:
            key = self._shelf.wait_for_key(valid_keys=["1", "2"])
            if key is None:
                continue
            self._current_shelf = int(key)
            logger.info(f"[FSM] Shelf selected: {self._current_shelf}")
            self._shelf.lcd_print(0, f"Shelf {self._current_shelf} selected")
            self._shelf.lcd_print(1, "Press B to confirm")

            # รอ B ยืนยัน
            confirm = self._shelf.wait_for_key(valid_keys=["B"], timeout=30.0)
            if confirm == "B":
                self._state = State.WAIT_FOR_IR
                return
            # ถ้า Timeout หรือกดผิดให้วนใหม่

    def _state_wait_for_ir(self) -> None:
        """รอ IR ตรวจพบอาหารบนชั้นที่เลือก หรือกด * เพื่อ Reset"""
        self._print_status(f"วางอาหารบนชั้น {self._current_shelf} (กด * เพื่อยกเลิก)")
        self._shelf.lcd_print(0, f"Shelf {self._current_shelf}: waiting")
        self._shelf.lcd_print(1, "Place food or [*]")

        while True:
            # ตรวจ * (Cancel / Reset)
            key = self._shelf.poll_key()
            if key == "*":
                logger.info("[FSM] RESET triggered by keypad *")
                self._state = State.RESET_ALL
                return

            # ตรวจ IR
            if self._shelf.ir_has_food(self._current_shelf):
                logger.info(f"[FSM] Food detected on shelf {self._current_shelf}")
                self._shelf.lcd_print(0, "Food detected!")
                self._shelf.lcd_print(1, "Enter table num")
                self._state = State.ENTER_TABLE
                return

            time.sleep(0.05)

    def _state_enter_table(self) -> None:
        """รับหมายเลขโต๊ะจาก Keypad แล้วกด C ยืนยัน"""
        self._print_status("กรอกหมายเลขโต๊ะ (1 หรือ 2) แล้วกด C")
        self._shelf.lcd_print(0, "Table number:")
        self._shelf.lcd_print(1, "[1] or [2] + C")

        while True:
            key = self._shelf.wait_for_key(valid_keys=["1", "2"])
            if key is None:
                continue
            table_id = int(key)
            self._shelf.lcd_print(0, f"Table {table_id} chosen")
            self._shelf.lcd_print(1, "Press C to confirm")

            confirm = self._shelf.wait_for_key(valid_keys=["C"], timeout=30.0)
            if confirm == "C":
                order = DeliveryOrder(shelf=self._current_shelf, table_id=table_id)
                self._orders.append(order)
                logger.info(f"[FSM] Order added: Shelf {self._current_shelf} → Table {table_id}")
                self._state = State.SHOW_LIST
                return

    def _state_show_list(self) -> None:
        """แสดงรายการคำสั่งทั้งหมด รอ A (เพิ่ม) หรือ # (ยืนยันส่ง)"""
        self._print_order_list()
        self._shelf.lcd_print(0, "A:Add  #:Deliver")
        self._shelf.lcd_print(1, f"{len(self._orders)} order(s)")

        key = self._shelf.wait_for_key(valid_keys=["A", "#"])
        if key == "A":
            self._state = State.SELECT_FLOOR
        elif key == "#":
            logger.info("[FSM] Delivery confirmed. Starting delivery sequence.")
            self._state = State.DELIVERING

    def _state_reset_all(self) -> None:
        """ล้างงานทั้งหมดและกลับสู่จุดเริ่มต้น"""
        logger.info("[FSM] Resetting all orders.")
        self._orders.clear()
        self._current_shelf = None
        self._shelf.lcd_print(0, "RESET DONE")
        self._shelf.lcd_print(1, "Ready...")
        time.sleep(1.0)
        self._state = State.SELECT_FLOOR

    def _state_delivering(self) -> None:
        """
        ส่งอาหารให้โต๊ะถัดไปจากคิว
        Sequence: Station → Junction → เลี้ยว → โต๊ะ
        """
        if not self._orders:
            # ไม่มีคิวแล้ว ให้ตรวจสอบผิดปกติ
            self._state = State.CHECK_REMAIN
            return

        # หยิบ order แรกจากคิว
        order = self._orders[0]
        target_y = TABLE1_Y if order.table_id == 1 else -TABLE2_Y
        turn_deg = +90.0 if order.table_id == 1 else -90.0

        self._print_status(f"กำลังส่งอาหาร → โต๊ะ {order.table_id} (ชั้น {order.shelf})")
        self._shelf.lcd_print(0, f"Delivering T{order.table_id}")
        self._shelf.lcd_print(1, f"Shelf {order.shelf}...")

        x, y, _ = self._odom.pose

        # 1. วิ่งตรงไปยัง Junction (ถ้ายังไม่ถึง)
        dist_to_junction = JUNCTION_X - x
        if dist_to_junction > 0.05:
            ok = self._motion.forward(dist_to_junction)
            if not ok:
                logger.error("[FSM] Failed to reach junction. Stopping.")
                self._motion.stop()
                return

        # 2. เลี้ยวเข้าทิศโต๊ะ
        ok = self._motion.turn(turn_deg)
        if not ok:
            logger.error(f"[FSM] Turn failed. Aborting delivery to table {order.table_id}.")
            self._motion.stop()
            return

        # 3. วิ่งตรงเข้าหาโต๊ะ
        ok = self._motion.forward(abs(target_y))
        if not ok:
            logger.error(f"[FSM] Forward to table {order.table_id} failed.")
            self._motion.stop()
            return

        logger.info(f"[FSM] Arrived at Table {order.table_id}.")
        self._shelf.lcd_print(0, f"Arrived T{order.table_id}!")
        self._shelf.lcd_print(1, "Wait pickup...")

        self._state = State.WAIT_PICKUP

    def _state_wait_pickup(self) -> None:
        """รอลูกค้าหยิบอาหารออก (IR ไม่พบ) หรือ Manual Override"""
        if not self._orders:
            self._state = State.CHECK_REMAIN
            return

        order = self._orders[0]
        self._print_status(f"รอลูกค้ารับอาหาร โต๊ะ {order.table_id} ชั้น {order.shelf}")
        self._shelf.lcd_print(0, "Waiting pickup...")
        self._shelf.lcd_print(1, "OVERRIDE btn=skip")

        picked = self._shelf.wait_for_food_removed(
            shelf=order.shelf,
            timeout=PICKUP_WAIT_TIMEOUT_S,
        )

        if picked:
            logger.info(f"[FSM] Food picked from shelf {order.shelf}. Order complete.")
            self._orders.pop(0)  # ลบ order ที่เสร็จแล้วออกจากคิว
        else:
            logger.warning(f"[FSM] Pickup timeout for table {order.table_id}. Skipping.")
            self._orders.pop(0)

        self._state = State.CHECK_REMAIN

    def _state_check_remain(self) -> None:
        """ตรวจสอบว่ายังมีคิวการส่งอยู่อีกไหม"""
        if self._orders:
            logger.info(f"[FSM] {len(self._orders)} order(s) remaining. Continuing delivery.")

            # U-Turn ที่โต๊ะนี้ แล้ววิ่งกลับไปที่ Junction เพื่อไปโต๊ะถัดไป
            next_order = self._orders[0]
            next_turn_deg = +90.0 if next_order.table_id == 1 else -90.0

            self._shelf.lcd_print(0, "Next delivery...")
            self._shelf.lcd_print(1, f"Table {next_order.table_id}")

            # U-Turn กลับทาง
            self._motion.u_turn()
            # วิ่งกลับ Junction  (ระยะเท่ากับที่เข้ามา ขึ้นอยู่กับ order ก่อนหน้า)
            prev_y = TABLE1_Y if (len(self._orders) < 2) else TABLE2_Y
            self._motion.forward(prev_y)
            # เลี้ยวเข้าซอยโต๊ะถัดไป
            self._motion.turn(next_turn_deg)

            self._state = State.DELIVERING
        else:
            logger.info("[FSM] All orders delivered. Returning to station.")
            self._state = State.RETURN_STATION

    def _state_return_station(self) -> None:
        """
        กลับ Serve Station
        Sequence: U-Turn → วิ่งกลับ Junction → เลี้ยวขวา 90° → วิ่งเข้า Station → U-Turn
        """
        self._print_status("กำลังกลับ Serve Station...")
        self._shelf.lcd_print(0, "Returning home...")
        self._shelf.lcd_print(1, "Please wait")

        x, y, _ = self._odom.pose

        # 1. U-Turn ที่โต๊ะสุดท้าย (หรือตำแหน่งปัจจุบัน)
        self._motion.u_turn()

        # 2. วิ่งกลับจาก Y-offset ไปยัง Junction (Y = 0)
        dist_back_y = abs(y)
        if dist_back_y > 0.05:
            self._motion.forward(dist_back_y)

        # 3. เลี้ยวขวา 90° มุ่งหน้ากลับ Station (−X direction)
        self._motion.turn(-90.0)

        # 4. วิ่งตรงกลับ Station (X = 0)
        dist_to_home = JUNCTION_X
        self._motion.forward(dist_to_home)

        # 5. U-Turn ที่ Station เพื่อหันหน้าออก (พร้อมรับงานรอบถัดไป)
        self._motion.u_turn()

        # 6. Reset Odometry กลับเป็น (0, 0, 0)
        self._odom.reset()

        logger.info("[FSM] Returned to Serve Station. Ready for next round.")
        self._shelf.lcd_print(0, "Home! Ready.")
        self._shelf.lcd_print(1, "")

        print("\n" + "=" * 55)
        print("  ✓ กลับถึง Serve Station — พร้อมรับงานรอบถัดไป")
        print("=" * 55 + "\n")

        self._state = State.SELECT_FLOOR

    # ------------------------------------------------------------------
    # Display Helpers
    # ------------------------------------------------------------------

    def _print_status(self, msg: str) -> None:
        print(f"\n[FSM:{self._state.name}] {msg}")

    def _print_order_list(self) -> None:
        print(f"\n[FSM] === Order List ({len(self._orders)} item(s)) ===")
        for i, o in enumerate(self._orders, 1):
            print(f"  {i}. Shelf {o.shelf} → Table {o.table_id}")
        print("  กด A = เพิ่มรายการ  |  # = ยืนยันส่งทั้งหมด")

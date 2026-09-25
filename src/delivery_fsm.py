#!/usr/bin/env python3
"""Main delivery FSM controlled by the local POS touchscreen."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum, auto

from config import ARRIVAL_TOLERANCE_M, JUNCTION_X, TABLE1_Y, TABLE2_Y
from motion_client import MotionClient
from odometry import Odometry
from pos_server import PosBridge
from shelf_client import ShelfClient

logger = logging.getLogger(__name__)


class State(Enum):
    WAIT_FOR_POS = auto()
    DELIVERING = auto()
    WAIT_PICKUP = auto()
    CHECK_REMAIN = auto()
    RETURN_STATION = auto()
    ERROR = auto()


@dataclass(frozen=True)
class DeliveryOrder:
    """One confirmed shelf-to-table delivery."""

    shelf: int
    table_id: int


class DeliveryFSM:
    """Owns every robot movement and consumes UI commands between movements."""

    def __init__(
        self,
        motion: MotionClient,
        odometry: Odometry,
        shelf: ShelfClient,
        pos_bridge: PosBridge,
        waypoint_controller=None,
    ) -> None:
        self._motion = motion
        self._odom = odometry
        self._shelf = shelf
        self._pos = pos_bridge
        self._waypoint_ctrl = waypoint_controller

        self._state = State.WAIT_FOR_POS
        self._orders: list[DeliveryOrder] = []
        self._mission_id: str | None = None
        self._current_order_index: int | None = None

    def run(self) -> None:
        logger.info("[FSM] POS-controlled Delivery FSM started.")
        print("\n" + "=" * 55)
        print("  Food Delivery Robot — POS Control")
        print("=" * 55)
        while True:
            self._step()

    def _step(self) -> None:
        match self._state:
            case State.WAIT_FOR_POS:
                self._state_wait_for_pos()
            case State.DELIVERING:
                self._state_delivering()
            case State.WAIT_PICKUP:
                self._state_wait_pickup()
            case State.CHECK_REMAIN:
                self._state_check_remain()
            case State.RETURN_STATION:
                self._state_return_station()
            case State.ERROR:
                self._state_error()

    def _state_wait_for_pos(self) -> None:
        mission = self._pos.take_mission(timeout=0.25)
        if mission is None:
            return

        self._mission_id = mission["mission_id"]
        self._orders = [
            DeliveryOrder(shelf=item["shelf"], table_id=item["table_id"])
            for item in mission["orders"]
        ]
        self._current_order_index = 0
        logger.info(
            "[FSM] Mission %s accepted with %d order(s)",
            self._mission_id,
            len(self._orders),
        )
        if self._waypoint_ctrl is not None and not self._waypoint_ctrl.begin_mission():
            detail = getattr(self._waypoint_ctrl, "last_error", None)
            self._latch_error(detail or "ระบบนำทางยังไม่พร้อมเริ่มภารกิจ")
            return
        self._state = State.DELIVERING

    def _state_delivering(self) -> None:
        if self._mission_id is None or self._current_order_index is None:
            self._latch_error("ไม่พบข้อมูลภารกิจที่กำลังส่ง")
            return
        if self._current_order_index >= len(self._orders):
            self._state = State.RETURN_STATION
            return

        order = self._orders[self._current_order_index]
        message = f"กำลังเดินทางไปโต๊ะ {order.table_id}"
        self._pos.set_state(
            "NAVIGATING",
            message=message,
            current_order_index=self._current_order_index,
        )
        self._print_status(
            f"{message} (ชั้น {order.shelf})"
        )
        self._shelf.lcd_print(0, f"Delivering T{order.table_id}")
        self._shelf.lcd_print(1, f"Shelf {order.shelf}...")

        try:
            reached = self._navigate_to_table(order, self._current_order_index)
        except Exception as exc:
            logger.exception("[FSM] Navigation raised an exception")
            self._latch_error(f"ระบบนำทางขัดข้อง: {exc}")
            return
        if not reached:
            self._latch_error(f"ไปไม่ถึงโต๊ะ {order.table_id}; หยุดหุ่นยนต์แล้ว")
            return

        self._shelf.lcd_print(0, f"Arrived T{order.table_id}!")
        self._shelf.lcd_print(1, "Waiting for pickup")
        self._state = State.WAIT_PICKUP

    def _navigate_to_table(self, order: DeliveryOrder, index: int) -> bool:
        target_y = TABLE1_Y if order.table_id == 1 else -TABLE2_Y
        target_heading = 90.0 if order.table_id == 1 else -90.0

        if self._waypoint_ctrl is not None:
            return self._waypoint_ctrl.go_to_table(order.table_id)

        if index == 0:
            x, y, heading = self._odom.pose
            dx, dy = JUNCTION_X - x, -y
            distance = math.hypot(dx, dy)
            if distance > ARRIVAL_TOLERANCE_M:
                desired = math.atan2(dy, dx)
                turn = self._wrap_angle(desired - heading)
                if abs(math.degrees(turn)) > 1.0 and not self._motion.turn(math.degrees(turn)):
                    return False
                if not self._motion.forward(distance):
                    return False
                heading = desired
            turn = self._wrap_angle(math.radians(target_heading) - heading)
            if abs(math.degrees(turn)) > 1.0 and not self._motion.turn(math.degrees(turn)):
                return False
            return self._motion.forward(abs(target_y))

        previous = self._orders[index - 1]
        previous_distance = TABLE1_Y if previous.table_id == 1 else TABLE2_Y
        previous_heading = math.radians(90.0 if previous.table_id == 1 else -90.0)
        if not self._motion.turn(180.0):
            return False
        if not self._motion.forward(previous_distance):
            return False

        heading_at_junction = self._wrap_angle(previous_heading + math.pi)
        turn = self._wrap_angle(math.radians(target_heading) - heading_at_junction)
        if abs(math.degrees(turn)) > 1.0 and not self._motion.turn(math.degrees(turn)):
            return False
        return self._motion.forward(abs(target_y))

    def _state_wait_pickup(self) -> None:
        if self._mission_id is None or self._current_order_index is None:
            self._latch_error("ไม่พบข้อมูลภารกิจระหว่างรอรับอาหาร")
            return
        if self._current_order_index >= len(self._orders):
            self._state = State.CHECK_REMAIN
            return

        order = self._orders[self._current_order_index]
        while self._shelf.consume_override():
            logger.warning("[FSM] Discarded a physical override pressed before arrival.")

        message = f"ถึงโต๊ะ {order.table_id} แล้ว กรุณายืนยันเมื่อรับอาหาร"
        self._pos.set_state(
            "WAITING_PICKUP",
            message=message,
            current_order_index=self._current_order_index,
        )
        self._print_status(
            f"รอผู้ใช้ยืนยันรับอาหาร โต๊ะ {order.table_id} ชั้น {order.shelf}"
        )
        self._shelf.lcd_print(0, f"Waiting T{order.table_id}")
        self._shelf.lcd_print(1, "Confirm on POS")

        expected = (self._mission_id, self._current_order_index)
        while True:
            if self._shelf.consume_override():
                logger.info("[FSM] Physical pickup override pressed.")
                break

            confirmation = self._pos.take_pickup_confirmation(timeout=0.1)
            if confirmation is None:
                continue
            if confirmation == expected:
                break
            logger.warning("[FSM] Ignored stale pickup confirmation.")

        logger.info("[FSM] Pickup confirmed for shelf %d.", order.shelf)
        self._state = State.CHECK_REMAIN

    def _state_check_remain(self) -> None:
        if self._current_order_index is None:
            self._latch_error("ไม่พบลำดับรายการส่ง")
            return

        next_index = self._current_order_index + 1
        if next_index < len(self._orders):
            self._current_order_index = next_index
            next_order = self._orders[next_index]
            self._shelf.lcd_print(0, "Next delivery...")
            self._shelf.lcd_print(1, f"Table {next_order.table_id}")
            self._state = State.DELIVERING
            return

        self._state = State.RETURN_STATION

    def _state_return_station(self) -> None:
        if self._mission_id is None or not self._orders:
            self._latch_error("ไม่พบข้อมูลสำหรับเดินทางกลับครัว")
            return

        self._pos.set_state(
            "RETURNING",
            message="ส่งครบแล้ว กำลังกลับครัว",
            current_order_index=self._current_order_index,
        )
        self._print_status("กำลังกลับ Serve Station")
        self._shelf.lcd_print(0, "Returning home...")
        self._shelf.lcd_print(1, "Please wait")

        try:
            if self._waypoint_ctrl is not None:
                reached = self._waypoint_ctrl.return_home()
            else:
                reached = self._return_with_discrete_motion()
        except Exception as exc:
            logger.exception("[FSM] Return navigation raised an exception")
            self._latch_error(f"ระบบนำทางกลับครัวขัดข้อง: {exc}")
            return

        if not reached:
            self._latch_error("กลับครัวไม่สำเร็จ; หยุดหุ่นยนต์แล้ว")
            return

        self._odom.reset()
        self._shelf.lcd_print(0, "Home! Ready.")
        self._shelf.lcd_print(1, "")
        self._pos.set_state(
            "COMPLETED",
            message="กลับถึงครัวแล้ว พร้อมรับงานรอบใหม่",
            current_order_index=self._current_order_index,
        )
        logger.info("[FSM] Mission %s completed.", self._mission_id)
        self._orders = []
        self._mission_id = None
        self._current_order_index = None
        self._state = State.WAIT_FOR_POS

    def _return_with_discrete_motion(self) -> bool:
        last_order = self._orders[-1]
        last_distance = TABLE1_Y if last_order.table_id == 1 else TABLE2_Y
        last_heading = math.radians(90.0 if last_order.table_id == 1 else -90.0)

        if not self._motion.turn(180.0):
            return False
        if not self._motion.forward(last_distance):
            return False

        heading_at_junction = self._wrap_angle(last_heading + math.pi)
        turn_to_kitchen = self._wrap_angle(math.pi - heading_at_junction)
        if not self._motion.turn(math.degrees(turn_to_kitchen)):
            return False
        if not self._motion.forward(JUNCTION_X):
            return False
        return self._motion.turn(-180.0)

    def _latch_error(self, message: str) -> None:
        logger.error("[FSM] %s", message)
        try:
            self._motion.stop_continuous()
        except Exception:
            logger.exception("[FSM] Continuous stop command failed")
        try:
            self._motion.stop()
        except Exception:
            logger.exception("[FSM] Stop command failed")
        self._pos.set_state("ERROR", message=message, error=message)
        self._state = State.ERROR

    def _state_error(self) -> None:
        if not self._pos.take_reset(timeout=0.25):
            return

        # The robot is already stopped by _latch_error. Keep stop commands here
        # as a second guard before clearing the latched mission.
        try:
            self._motion.stop_continuous()
        except Exception:
            logger.exception("[FSM] Continuous stop command failed during reset")
        try:
            self._motion.stop()
        except Exception:
            logger.exception("[FSM] Stop command failed during reset")
        self._orders = []
        self._mission_id = None
        self._current_order_index = None
        self._pos.set_state("IDLE", message="รีเซ็ตแล้ว พร้อมเริ่มงานใหม่")
        self._state = State.WAIT_FOR_POS
        logger.info("[FSM] Error mission reset; ready for a new mission.")

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _print_status(message: str) -> None:
        print(f"\n[FSM] {message}")

#!/usr/bin/env python3
"""
main.py — Food Delivery Robot Entry Point
==========================================
จุดเริ่มต้นโปรแกรม: เชื่อมต่อ Serial, เริ่ม Background Threads แล้ว run Main FSM

การรัน:
    python3 main.py

ตัวเลือก Environment Variable:
    MOTION_PORT  — Serial port ของ Arduino #1  (default: /dev/ttyUSB0)
    SHELF_PORT   — Serial port ของ Arduino #2  (default: /dev/ttyUSB1)
    BAUD_RATE    — Baud rate ทั้งสอง port       (default: 115200)
    LOG_LEVEL    — DEBUG / INFO / WARNING        (default: INFO)
"""

import logging
import os
import sys
import signal
import serial

# Local imports (ต้องรันจาก src/ หรือเพิ่ม src/ ใน PYTHONPATH)
from config import (
    MOTION_SERIAL_PORT,
    SHELF_SERIAL_PORT,
    SERIAL_BAUD,
    SERIAL_TIMEOUT,
)
from odometry import Odometry
from motion_client import MotionClient
from shelf_client import ShelfClient
from delivery_fsm import DeliveryFSM


# ===========================================================
# Logging Setup
# ===========================================================
def _setup_logging() -> None:
    level_str = os.environ.get("LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_str, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    )


# ===========================================================
# Serial Connection Helper
# ===========================================================
def _open_serial(port: str, baud: int, timeout: float, label: str) -> serial.Serial:
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        logging.getLogger(__name__).info(f"[{label}] Connected: {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        logging.getLogger(__name__).critical(f"[{label}] Cannot open {port}: {e}")
        sys.exit(1)


# ===========================================================
# Graceful Shutdown Handler
# ===========================================================
def _make_shutdown_handler(odom: Odometry, shelf: ShelfClient, motion: MotionClient):
    def _handler(sig, frame):
        print("\n[main] Shutting down... sending STOP")
        motion.stop()
        odom.stop()
        shelf.stop()
        sys.exit(0)
    return _handler


# ===========================================================
# Main
# ===========================================================
def main() -> None:
    _setup_logging()
    logger = logging.getLogger(__name__)

    # --- Read port overrides from env ---
    motion_port = os.environ.get("MOTION_PORT", MOTION_SERIAL_PORT)
    shelf_port  = os.environ.get("SHELF_PORT",  SHELF_SERIAL_PORT)
    baud        = int(os.environ.get("BAUD_RATE", SERIAL_BAUD))

    logger.info("=" * 55)
    logger.info("  Food Delivery Robot — Booting up")
    logger.info("=" * 55)
    logger.info(f"  Motion Arduino : {motion_port}")
    logger.info(f"  Shelf Arduino  : {shelf_port}")
    logger.info(f"  Baud Rate      : {baud}")

    # --- Open Serial Connections ---
    motion_ser = _open_serial(motion_port, baud, SERIAL_TIMEOUT, "Motion")
    shelf_ser  = _open_serial(shelf_port,  baud, SERIAL_TIMEOUT, "Shelf")

    # --- Instantiate Subsystems ---
    odometry = Odometry(motion_ser)
    motion   = MotionClient(motion_ser)
    shelf    = ShelfClient(shelf_ser)

    # --- Register Ctrl+C Shutdown ---
    signal.signal(signal.SIGINT, _make_shutdown_handler(odometry, shelf, motion))

    # --- Start Background Threads ---
    odometry.start()
    shelf.start()

    logger.info("[main] All subsystems started. Launching Main FSM.")

    # --- Run Main FSM (blocks forever) ---
    fsm = DeliveryFSM(motion=motion, odometry=odometry, shelf=shelf)
    try:
        fsm.run()
    except Exception as e:
        logger.exception(f"[main] Unhandled FSM exception: {e}")
        motion.stop()
    finally:
        logger.info("[main] Cleaning up...")
        odometry.stop()
        shelf.stop()
        motion_ser.close()
        shelf_ser.close()
        logger.info("[main] Shutdown complete.")


if __name__ == "__main__":
    main()

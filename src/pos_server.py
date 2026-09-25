#!/usr/bin/env python3
"""Small stdlib-only HTTP server and thread-safe command bridge for the POS."""

from __future__ import annotations

import json
import logging
import mimetypes
import os
import queue
import threading
import uuid
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

logger = logging.getLogger(__name__)

MAX_REQUEST_BYTES = 16 * 1024
STATIC_FILES = {"index.html", "app.css", "app.js"}


class PosRequestError(Exception):
    """An expected request rejection with an HTTP status and user-facing message."""

    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


class PosBridge:
    """Owns the API snapshot and queues commands for the main FSM thread."""

    def __init__(self) -> None:
        self.missions: queue.Queue[dict] = queue.Queue(maxsize=1)
        self.pickup_confirmations: queue.Queue[tuple[str, int]] = queue.Queue()
        self.resets: queue.Queue[bool] = queue.Queue(maxsize=1)
        self._lock = threading.RLock()
        self.cancel_event = threading.Event()
        self._cancel_handler = None
        self._pickup_pending: tuple[str, int] | None = None
        self._draft: dict[int, dict] = {
            1: {"table_id": None, "loaded_confirmed": False},
            2: {"table_id": None, "loaded_confirmed": False},
        }
        self._active_shelf = 1
        self._keypad_sidebar_open = False
        self._keypad_sidebar_pending = False
        self._keypad_sidebar_request = 0
        self._setup_message = "เลือกชั้นและโต๊ะเพื่อเริ่มงาน"
        self._snapshot: dict = {
            "state": "IDLE",
            "mission_id": None,
            "orders": [],
            "current_order_index": None,
            "message": "พร้อมรับงาน",
            "error": None,
            "obstacle_detected": False,
        }

    def snapshot(self) -> dict:
        with self._lock:
            return {
                **self._snapshot,
                "orders": [dict(order) for order in self._snapshot["orders"]],
                "draft": {
                    str(shelf): dict(selection)
                    for shelf, selection in self._draft.items()
                },
                "active_shelf": self._active_shelf,
                "keypad_sidebar_request": self._keypad_sidebar_request,
                "keypad_sidebar_open": self._keypad_sidebar_open,
                "keypad_sidebar_pending": self._keypad_sidebar_pending,
                "setup_message": self._setup_message,
            }

    def set_obstacle_detected(self, detected: bool) -> None:
        """Publish the LiDAR safety pause state to the POS screen."""
        with self._lock:
            self._snapshot["obstacle_detected"] = bool(detected)

    def set_cancel_handler(self, handler) -> None:
        """Register the motion controller's immediate stop callback."""
        self._cancel_handler = handler

    def request_cancel(self) -> dict:
        """Request a safe stop for a running mission."""
        with self._lock:
            state = self._snapshot["state"]
            if state == "CANCELLING":
                return {"accepted": True}
            if state not in {"PREPARING", "NAVIGATING", "WAITING_PICKUP", "PICKUP_DELAY", "RETURNING"}:
                raise PosRequestError(409, "ไม่มีภารกิจที่กำลังทำงานให้ยกเลิก")

            self.cancel_event.set()
            self._snapshot = {
                **self._snapshot,
                "state": "CANCELLING",
                "message": "กำลังหยุดหุ่นยนต์...",
                "error": None,
            }
            if state == "PREPARING":
                try:
                    self.missions.get_nowait()
                except queue.Empty:
                    pass
            handler = self._cancel_handler

        if handler is not None:
            handler()
        return {"accepted": True}

    def complete_cancel(self) -> None:
        """Return a stopped, cancelled mission to the ready POS state."""
        with self._lock:
            self._snapshot = {
                **self._snapshot,
                "state": "IDLE",
                "mission_id": None,
                "orders": [],
                "current_order_index": None,
                "message": "ยกเลิกภารกิจแล้ว หุ่นยนต์หยุดแล้ว",
                "error": None,
            }
            self._clear_draft_locked()
            self._setup_message = "ยกเลิกภารกิจแล้ว หุ่นยนต์หยุดแล้ว"
            self._pickup_pending = None
            self.cancel_event.clear()
            self._snapshot["obstacle_detected"] = False

    def apply_pos_action(
        self,
        action: object,
        *,
        shelf: object = None,
        table_id: object = None,
    ) -> dict:
        """Apply one setup action shared by the touchscreen and keypad."""
        if not isinstance(action, str):
            raise PosRequestError(400, "ไม่พบคำสั่ง POS")

        action = action.strip().lower()
        if action == "start":
            return self._submit_draft_mission()

        with self._lock:
            if self._snapshot["state"] not in {"IDLE", "COMPLETED"}:
                raise PosRequestError(409, "แก้รายการได้เฉพาะขณะหุ่นยนต์รอรับงาน")

            if action in {"sidebar_open", "sidebar_close"}:
                self._keypad_sidebar_open = action == "sidebar_open"
                self._keypad_sidebar_pending = False
                return {"accepted": True}

            if action == "clear_all":
                self._clear_draft_locked()
                self._setup_message = "ล้างรายการทั้งหมดแล้ว"
                return {"accepted": True}

            if type(shelf) is not int or shelf not in (1, 2):
                raise PosRequestError(400, "ชั้นต้องเป็น 1 หรือ 2")
            self._active_shelf = shelf
            selection = self._draft[shelf]

            if action == "select_shelf":
                self._setup_message = f"เลือกชั้น {shelf} แล้ว"
            elif action == "set_table":
                if type(table_id) is not int or table_id not in (1, 2):
                    raise PosRequestError(400, "โต๊ะต้องเป็น 1 หรือ 2")
                if selection["table_id"] == table_id:
                    selection["table_id"] = None
                    selection["loaded_confirmed"] = False
                    self._setup_message = f"ยกเลิกโต๊ะของชั้น {shelf} แล้ว"
                else:
                    selection["table_id"] = table_id
                    selection["loaded_confirmed"] = True
                    self._setup_message = f"เลือกโต๊ะ {table_id} สำหรับชั้น {shelf} แล้ว"
            elif action == "toggle_loaded":
                if selection["table_id"] is None:
                    raise PosRequestError(409, f"กรุณาเลือกโต๊ะให้ชั้น {shelf} ก่อน")
                selection["loaded_confirmed"] = not selection["loaded_confirmed"]
                self._setup_message = (
                    f"ยืนยันแล้วว่าวางอาหารบนชั้น {shelf}"
                    if selection["loaded_confirmed"]
                    else f"ยกเลิกการยืนยันชั้น {shelf}"
                )
            elif action == "clear_shelf":
                self._draft[shelf] = {
                    "table_id": None,
                    "loaded_confirmed": False,
                }
                self._setup_message = f"ล้างรายการชั้น {shelf} แล้ว"
            else:
                raise PosRequestError(400, "คำสั่ง POS ไม่ถูกต้อง")

            return {"accepted": True}

    def handle_keypad_key(self, raw_key: object) -> dict:
        """Map a physical keypad key onto the current POS state.

        This callback never raises because it runs from a serial/ROS worker.
        Rejected keys are logged and returned to the caller for tests.
        """
        if not isinstance(raw_key, str):
            return {"accepted": False, "reason": "invalid_key"}
        key = raw_key.strip().upper()
        if len(key) != 1 or key not in "0123456789ABCD*#":
            return {"accepted": False, "reason": "invalid_key"}

        try:
            with self._lock:
                state = self._snapshot["state"]
                active_shelf = self._active_shelf
                mission_id = self._snapshot["mission_id"]
                order_index = self._snapshot["current_order_index"]
                keypad_sidebar_open = self._keypad_sidebar_open

            if state in {"IDLE", "COMPLETED"}:
                if key == "A":
                    result = self.apply_pos_action("select_shelf", shelf=2)
                    with self._lock:
                        self._keypad_sidebar_pending = not self._keypad_sidebar_open
                        self._keypad_sidebar_request += 1
                elif key == "B":
                    result = self.apply_pos_action("select_shelf", shelf=1)
                    with self._lock:
                        self._keypad_sidebar_pending = not self._keypad_sidebar_open
                        self._keypad_sidebar_request += 1
                elif not keypad_sidebar_open:
                    return {"accepted": False, "reason": "sidebar_closed"}
                elif key in {"1", "2"}:
                    result = self.apply_pos_action(
                        "set_table",
                        shelf=active_shelf,
                        table_id=int(key),
                    )
                elif key == "C":
                    result = self.apply_pos_action(
                        "toggle_loaded",
                        shelf=active_shelf,
                    )
                elif key == "D":
                    result = self.apply_pos_action(
                        "clear_shelf",
                        shelf=active_shelf,
                    )
                elif key == "*":
                    result = self.apply_pos_action("clear_all")
                elif key == "#":
                    result = self.apply_pos_action("start")
                else:
                    return {"accepted": False, "reason": "unused_key"}
            elif state == "WAITING_PICKUP" and key == "#":
                result = self.confirm_pickup(mission_id, order_index)
            elif state == "ERROR" and key == "*":
                result = self.request_reset()
            else:
                return {"accepted": False, "reason": "ignored_in_state"}

            logger.info("Accepted keypad key %s in POS state %s", key, state)
            response = dict(result)
            response.setdefault("accepted", True)
            return response
        except PosRequestError as exc:
            logger.warning("Rejected keypad key %s: %s", key, exc.message)
            return {"accepted": False, "reason": exc.message}

    def _submit_draft_mission(self) -> dict:
        with self._lock:
            orders = [
                {
                    "shelf": shelf,
                    "table_id": selection["table_id"],
                    "loaded_confirmed": selection["loaded_confirmed"],
                }
                for shelf, selection in self._draft.items()
                if selection["table_id"] is not None
            ]
        return self.submit_mission(orders)

    def _clear_draft_locked(self) -> None:
        self._draft = {
            1: {"table_id": None, "loaded_confirmed": False},
            2: {"table_id": None, "loaded_confirmed": False},
        }
        self._active_shelf = 1

    def submit_mission(self, raw_orders: object) -> dict:
        orders = self._validate_orders(raw_orders)
        with self._lock:
            if self._snapshot["state"] not in {"IDLE", "COMPLETED"}:
                raise PosRequestError(409, "หุ่นยนต์กำลังทำงานหรือรอการกู้คืน")

            mission_id = uuid.uuid4().hex
            mission = {"mission_id": mission_id, "orders": orders}
            try:
                self.missions.put_nowait(mission)
            except queue.Full as exc:
                raise PosRequestError(409, "มีงานรอเริ่มอยู่แล้ว") from exc

            self._pickup_pending = None
            self.cancel_event.clear()
            self._keypad_sidebar_open = False
            self._keypad_sidebar_pending = False
            self._snapshot = {
                "state": "PREPARING",
                "mission_id": mission_id,
                "orders": [dict(order) for order in orders],
                "current_order_index": None,
                "message": "รับรายการแล้ว กำลังเตรียมหุ่นยนต์",
                "error": None,
                "obstacle_detected": False,
            }
            self._clear_draft_locked()
            self._setup_message = "รับรายการแล้ว กำลังเริ่มภารกิจ"
            logger.info("Accepted POS mission %s with %d order(s)", mission_id, len(orders))
            return {"mission_id": mission_id, "state": "PREPARING"}

    @staticmethod
    def _validate_orders(raw_orders: object) -> list[dict]:
        if not isinstance(raw_orders, list) or not 1 <= len(raw_orders) <= 2:
            raise PosRequestError(400, "ต้องระบุรายการส่งอาหาร 1 ถึง 2 รายการ")

        orders: list[dict] = []
        seen_shelves: set[int] = set()
        for item in raw_orders:
            if not isinstance(item, dict):
                raise PosRequestError(400, "รูปแบบรายการส่งอาหารไม่ถูกต้อง")

            shelf = item.get("shelf")
            table_id = item.get("table_id")
            loaded_confirmed = item.get("loaded_confirmed")
            if type(shelf) is not int or shelf not in (1, 2):
                raise PosRequestError(400, "ชั้นต้องเป็น 1 หรือ 2")
            if type(table_id) is not int or table_id not in (1, 2):
                raise PosRequestError(400, "โต๊ะต้องเป็น 1 หรือ 2")
            if loaded_confirmed is not True:
                raise PosRequestError(400, f"กรุณายืนยันว่าของวางบนชั้น {shelf} แล้ว")
            if shelf in seen_shelves:
                raise PosRequestError(400, f"เลือกชั้น {shelf} ซ้ำไม่ได้")

            seen_shelves.add(shelf)
            orders.append(
                {
                    "shelf": shelf,
                    "table_id": table_id,
                    "loaded_confirmed": True,
                }
            )

        return sorted(orders, key=lambda order: order["shelf"])

    def take_mission(self, timeout: float) -> dict | None:
        try:
            return self.missions.get(timeout=timeout)
        except queue.Empty:
            return None

    def set_state(
        self,
        state: str,
        *,
        message: str,
        current_order_index: int | None = None,
        error: str | None = None,
    ) -> None:
        with self._lock:
            if self.cancel_event.is_set() and state in {
                "PREPARING", "NAVIGATING", "WAITING_PICKUP", "PICKUP_DELAY", "RETURNING"
            }:
                state = "CANCELLING"
                message = "กำลังหยุดหุ่นยนต์..."
            self._snapshot = {
                **self._snapshot,
                "state": state,
                "current_order_index": current_order_index,
                "message": message,
                "error": error,
            }
            if state != "WAITING_PICKUP":
                self._pickup_pending = None
            if state in {"IDLE", "COMPLETED"}:
                self._setup_message = message

    def confirm_pickup(self, mission_id: object, order_index: object) -> dict:
        with self._lock:
            snapshot = self._snapshot
            if snapshot["state"] != "WAITING_PICKUP":
                raise PosRequestError(409, "หุ่นยนต์ไม่ได้รอการยืนยันรับอาหาร")
            if mission_id != snapshot["mission_id"]:
                raise PosRequestError(409, "รายการยืนยันนี้เป็นของภารกิจเก่า")
            if type(order_index) is not int or order_index != snapshot["current_order_index"]:
                raise PosRequestError(409, "รายการยืนยันนี้ไม่ตรงกับโต๊ะปัจจุบัน")

            confirmation = (mission_id, order_index)
            if self._pickup_pending == confirmation:
                raise PosRequestError(409, "รายการนี้ได้รับการยืนยันแล้ว")

            self._pickup_pending = confirmation
            self.pickup_confirmations.put_nowait(confirmation)
            logger.info(
                "Pickup confirmed for mission %s order %d",
                mission_id,
                order_index,
            )
            return {"accepted": True}

    def take_pickup_confirmation(self, timeout: float) -> tuple[str, int] | None:
        try:
            return self.pickup_confirmations.get(timeout=timeout)
        except queue.Empty:
            return None

    def request_reset(self) -> dict:
        with self._lock:
            if self._snapshot["state"] != "ERROR":
                raise PosRequestError(409, "รีเซ็ตได้เมื่อหุ่นยนต์หยุดจากข้อผิดพลาดเท่านั้น")
            try:
                self.resets.put_nowait(True)
            except queue.Full as exc:
                raise PosRequestError(409, "กำลังรีเซ็ตหุ่นยนต์") from exc
            return {"accepted": True}

    def take_reset(self, timeout: float) -> bool:
        try:
            self.resets.get(timeout=timeout)
            return True
        except queue.Empty:
            return False


class PosRequestHandler(BaseHTTPRequestHandler):
    server_version = "RobotPOS/1.0"
    sys_version = ""

    def __init__(self, *args, bridge: PosBridge, static_root: Path, **kwargs) -> None:
        self.bridge = bridge
        self.static_root = static_root.resolve()
        super().__init__(*args, **kwargs)

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/api/config":
            theme = os.environ.get("POS_THEME", "light").strip().lower()
            self._send_json(200, {"theme": theme if theme in {"light", "dark"} else "light"})
            return
        if path == "/api/health":
            self._send_json(200, {"ok": True})
            return
        if path == "/api/state":
            self._send_json(200, self.bridge.snapshot())
            return
        self._serve_static(path)

    def do_POST(self) -> None:
        origin = self.headers.get("Origin")
        if not self._is_local_origin(origin):
            self._send_json(403, {"error": "origin ไม่ได้รับอนุญาต"})
            return

        try:
            payload = self._read_json()
            path = urlsplit(self.path).path
            if path == "/api/mission/start":
                response = self.bridge.submit_mission(payload.get("orders") if isinstance(payload, dict) else None)
                self._send_json(202, response)
            elif path == "/api/pos/action":
                if not isinstance(payload, dict):
                    raise PosRequestError(400, "รูปแบบคำขอไม่ถูกต้อง")
                response = self.bridge.apply_pos_action(
                    payload.get("action"),
                    shelf=payload.get("shelf"),
                    table_id=payload.get("table_id"),
                )
                self._send_json(202, response)
            elif path == "/api/mission/pickup-confirmed":
                if not isinstance(payload, dict):
                    raise PosRequestError(400, "รูปแบบคำขอไม่ถูกต้อง")
                response = self.bridge.confirm_pickup(
                    payload.get("mission_id"),
                    payload.get("order_index"),
                )
                self._send_json(202, response)
            elif path == "/api/mission/reset":
                response = self.bridge.request_reset()
                self._send_json(202, response)
            elif path == "/api/mission/cancel":
                response = self.bridge.request_cancel()
                self._send_json(202, response)
            else:
                self._send_json(404, {"error": "ไม่พบ API"})
        except PosRequestError as exc:
            self._send_json(exc.status, {"error": exc.message})
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"error": "ข้อมูล JSON ไม่ถูกต้อง"})
        except Exception:
            logger.exception("Unhandled POS request error")
            self._send_json(500, {"error": "เกิดข้อผิดพลาดภายในระบบ"})

    @staticmethod
    def _is_local_origin(origin: str | None) -> bool:
        """Allow browser pages served through a loopback SSH forward.

        The browser's local forwarded port can differ from the server port,
        and users may open either localhost or 127.0.0.1. The server itself
        remains bound to loopback, and non-loopback web origins stay blocked.
        """
        if not origin:
            return False
        try:
            parsed = urlsplit(origin)
            return (
                parsed.scheme == "http"
                and parsed.hostname in {"127.0.0.1", "localhost", "::1"}
                and parsed.username is None
                and parsed.password is None
                and parsed.path == ""
                and parsed.query == ""
                and parsed.fragment == ""
            )
        except ValueError:
            return False

    def _read_json(self) -> object:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise PosRequestError(400, "Content-Length ไม่ถูกต้อง") from exc
        if length <= 0:
            raise PosRequestError(400, "ไม่พบข้อมูลคำขอ")
        if length > MAX_REQUEST_BYTES:
            raise PosRequestError(413, "ข้อมูลคำขอใหญ่เกินกำหนด")
        content_type = self.headers.get_content_type()
        if content_type != "application/json":
            raise PosRequestError(415, "ต้องส่งข้อมูลแบบ application/json")
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def _serve_static(self, request_path: str) -> None:
        relative = unquote(request_path).lstrip("/") or "index.html"
        if relative not in STATIC_FILES:
            self._send_json(404, {"error": "ไม่พบไฟล์"})
            return
        target = (self.static_root / relative).resolve()
        if target != self.static_root and self.static_root not in target.parents:
            self._send_json(404, {"error": "ไม่พบไฟล์"})
            return
        if not target.is_file():
            self._send_json(404, {"error": "ไม่พบไฟล์"})
            return

        content_type, _ = mimetypes.guess_type(target.name)
        content_type = content_type or "application/octet-stream"
        if content_type.startswith("text/") or content_type in {
            "application/javascript",
            "application/json",
        }:
            content_type += "; charset=utf-8"
        try:
            data = target.read_bytes()
        except OSError:
            self._send_json(404, {"error": "ไม่พบไฟล์"})
            return

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self._security_headers()
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, status: int, payload: dict) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self._security_headers()
        self.end_headers()
        self.wfile.write(data)

    def _security_headers(self) -> None:
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; connect-src 'self'; img-src 'self' data:; "
            "style-src 'self'; script-src 'self'; object-src 'none'; base-uri 'none'",
        )

    def log_message(self, format: str, *args) -> None:
        logger.info("POS HTTP: " + format, *args)


class PosServer:
    """Lifecycle wrapper for a local-only POS HTTP server."""

    def __init__(
        self,
        bridge: PosBridge,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        static_root: Path | None = None,
    ) -> None:
        self.bridge = bridge
        self.host = host
        self.port = port
        self.static_root = static_root or Path(__file__).resolve().parent.parent / "pos"
        handler = partial(
            PosRequestHandler,
            bridge=bridge,
            static_root=self.static_root,
        )
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._httpd.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self._httpd.server_port}/"

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="pos-http-server",
            daemon=True,
        )
        self._thread.start()
        logger.info("POS server listening at %s", self.url)

    def stop(self) -> None:
        if self._thread is not None:
            self._httpd.shutdown()
            self._thread.join(timeout=3.0)
            self._thread = None
        self._httpd.server_close()

#!/usr/bin/env python3
"""Small stdlib-only HTTP server and thread-safe command bridge for the POS."""

from __future__ import annotations

import json
import logging
import mimetypes
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
        self._pickup_pending: tuple[str, int] | None = None
        self._snapshot: dict = {
            "state": "IDLE",
            "mission_id": None,
            "orders": [],
            "current_order_index": None,
            "message": "พร้อมรับงาน",
            "error": None,
        }

    def snapshot(self) -> dict:
        with self._lock:
            return {
                **self._snapshot,
                "orders": [dict(order) for order in self._snapshot["orders"]],
            }

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
            self._snapshot = {
                "state": "PREPARING",
                "mission_id": mission_id,
                "orders": [dict(order) for order in orders],
                "current_order_index": None,
                "message": "รับรายการแล้ว กำลังเตรียมหุ่นยนต์",
                "error": None,
            }
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
            self._snapshot = {
                **self._snapshot,
                "state": state,
                "current_order_index": current_order_index,
                "message": message,
                "error": error,
            }
            if state != "WAITING_PICKUP":
                self._pickup_pending = None

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

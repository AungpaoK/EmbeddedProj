#!/usr/bin/env python3
"""Interactive terminal client for the local food-delivery POS API."""

from __future__ import annotations

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class PosApiError(RuntimeError):
    pass


class PosApi:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.origin = self.base_url

    def state(self) -> dict:
        return self._request("GET", "/api/state")

    def submit(self, orders: list[dict]) -> dict:
        return self._request("POST", "/api/mission/start", {"orders": orders})

    def confirm_pickup(self, mission_id: str, order_index: int) -> dict:
        return self._request(
            "POST",
            "/api/mission/pickup-confirmed",
            {"mission_id": mission_id, "order_index": order_index},
        )

    def _request(self, method: str, path: str, payload: dict | None = None) -> dict:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
            headers["Origin"] = self.origin
        request = Request(
            self.base_url + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urlopen(request, timeout=3.0) as response:
                return json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            try:
                body = json.loads(error.read().decode("utf-8"))
                message = body.get("error", f"HTTP {error.code}")
            except Exception:
                message = f"HTTP {error.code}"
            finally:
                error.close()
            raise PosApiError(message) from error
        except (URLError, OSError, json.JSONDecodeError) as error:
            raise PosApiError(f"เชื่อมต่อ POS ไม่สำเร็จ: {error}") from error


def _choose(prompt: str, allowed: set[str]) -> str:
    while True:
        answer = input(prompt).strip().lower()
        if answer in allowed:
            return answer
        print(f"กรุณาเลือก: {', '.join(sorted(allowed))}")


def _build_mission() -> list[dict] | None:
    count = int(_choose("จำนวนชั้นที่จะส่ง (1/2): ", {"1", "2"}))
    if count == 1:
        shelves = [int(_choose("เลือกชั้นวางอาหาร (1/2): ", {"1", "2"}))]
    else:
        shelves = [1, 2]

    orders: list[dict] = []
    for shelf in shelves:
        table_id = int(_choose(f"ชั้น {shelf} ส่งโต๊ะ (1/2): ", {"1", "2"}))
        loaded = _choose(f"ยืนยันว่าใส่อาหารบนชั้น {shelf} แล้ว? (y/n): ", {"y", "n"})
        if loaded != "y":
            print("ยกเลิกการสร้างภารกิจ: ยังไม่ได้ยืนยันอาหาร")
            return None
        orders.append(
            {"shelf": shelf, "table_id": table_id, "loaded_confirmed": True}
        )

    print("\nรายการส่ง:")
    for order in orders:
        print(f"  ชั้น {order['shelf']} -> โต๊ะ {order['table_id']}")
    if _choose("เริ่มภารกิจนี้หรือไม่? (y/n): ", {"y", "n"}) != "y":
        print("ยกเลิกแล้ว")
        return None
    return orders


def _print_state(snapshot: dict) -> None:
    print("\n" + "=" * 56)
    print(f"สถานะ: {snapshot.get('state', 'UNKNOWN')}")
    print(snapshot.get("message") or "")
    orders = snapshot.get("orders") or []
    current = snapshot.get("current_order_index")
    for index, order in enumerate(orders):
        marker = ">" if index == current else " "
        print(f"{marker} ชั้น {order['shelf']} -> โต๊ะ {order['table_id']}")
    if snapshot.get("error"):
        print(f"ข้อผิดพลาด: {snapshot['error']}")
    print("=" * 56)


def run_console(api: PosApi) -> None:
    print("Food Delivery Robot — Terminal Console")
    while True:
        try:
            snapshot = api.state()
            _print_state(snapshot)
            state = snapshot.get("state")

            if state in {"IDLE", "COMPLETED"}:
                action = _choose("[n] งานใหม่  [r] รีเฟรช  [q] ออก: ", {"n", "r", "q"})
                if action == "n":
                    orders = _build_mission()
                    if orders:
                        result = api.submit(orders)
                        print(f"รับภารกิจแล้ว: {result['mission_id']}")
                elif action == "q":
                    return
            elif state == "WAITING_PICKUP":
                action = _choose("[c] ยืนยันรับอาหาร  [r] รีเฟรช  [q] ออก: ", {"c", "r", "q"})
                if action == "c":
                    api.confirm_pickup(
                        snapshot["mission_id"], snapshot["current_order_index"]
                    )
                    print("ยืนยันการรับอาหารแล้ว")
                elif action == "q":
                    return
            else:
                action = _choose("[r] รีเฟรชสถานะ  [q] ออก: ", {"r", "q"})
                if action == "q":
                    return
        except PosApiError as error:
            print(f"ข้อผิดพลาด: {error}")
            if _choose("[r] ลองใหม่  [q] ออก: ", {"r", "q"}) == "q":
                return


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive local POS console")
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    args = parser.parse_args()
    try:
        run_console(PosApi(args.url))
    except (KeyboardInterrupt, EOFError):
        print("\nออกจาก console")
    except BrokenPipeError:
        sys.exit(0)


if __name__ == "__main__":
    main()

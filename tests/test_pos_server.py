import json
import sys
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from pos_server import PosBridge, PosServer
from pos_console import PosApi, PosApiError


class PosServerTests(unittest.TestCase):
    def setUp(self):
        self.bridge = PosBridge()
        self.server = PosServer(self.bridge, port=0)
        self.server.start()
        self.origin = self.server.url.rstrip("/")

    def tearDown(self):
        self.server.stop()

    def get_json(self, path):
        with urlopen(self.origin + path, timeout=2) as response:
            return response.status, json.loads(response.read().decode("utf-8"))

    def post_json(self, path, payload, *, origin=None):
        body = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if origin is not None:
            headers["Origin"] = origin
        request = Request(self.origin + path, data=body, headers=headers, method="POST")
        try:
            with urlopen(request, timeout=2) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            try:
                return error.code, json.loads(error.read().decode("utf-8"))
            finally:
                error.close()

    def test_serves_state_health_and_static_html(self):
        status, health = self.get_json("/api/health")
        self.assertEqual(status, 200)
        self.assertEqual(health, {"ok": True})

        status, state = self.get_json("/api/state")
        self.assertEqual(status, 200)
        self.assertEqual(state["state"], "IDLE")

        with urlopen(self.origin + "/", timeout=2) as response:
            html = response.read().decode("utf-8")
        self.assertIn("จัดรายการส่งอาหาร", html)
        self.assertIn("/app.js", html)

    def test_rejects_unconfirmed_or_duplicate_shelves(self):
        status, body = self.post_json(
            "/api/mission/start",
            {"orders": [{"shelf": 1, "table_id": 1, "loaded_confirmed": False}]},
            origin=self.origin,
        )
        self.assertEqual(status, 400)
        self.assertIn("ยืนยัน", body["error"])

        status, _ = self.post_json(
            "/api/mission/start",
            {
                "orders": [
                    {"shelf": 1, "table_id": 1, "loaded_confirmed": True},
                    {"shelf": 1, "table_id": 2, "loaded_confirmed": True},
                ]
            },
            origin=self.origin,
        )
        self.assertEqual(status, 400)
        self.assertEqual(self.bridge.snapshot()["state"], "IDLE")

    def test_accepts_orders_sorted_by_shelf_and_rejects_concurrent_mission(self):
        status, result = self.post_json(
            "/api/mission/start",
            {
                "orders": [
                    {"shelf": 2, "table_id": 2, "loaded_confirmed": True},
                    {"shelf": 1, "table_id": 1, "loaded_confirmed": True},
                ]
            },
            origin=self.origin,
        )
        self.assertEqual(status, 202)
        self.assertTrue(result["mission_id"])
        _, snapshot = self.get_json("/api/state")
        self.assertEqual(snapshot["state"], "PREPARING")
        self.assertEqual(snapshot["mission_id"], result["mission_id"])
        self.assertEqual([item["shelf"] for item in snapshot["orders"]], [1, 2])

        status, _ = self.post_json(
            "/api/mission/start",
            {"orders": [{"shelf": 1, "table_id": 1, "loaded_confirmed": True}]},
            origin=self.origin,
        )
        self.assertEqual(status, 409)

    def test_pickup_confirmation_is_bound_to_current_mission_and_order(self):
        mission = self.bridge.submit_mission(
            [{"shelf": 1, "table_id": 2, "loaded_confirmed": True}]
        )
        queued = self.bridge.take_mission(timeout=0.1)
        self.assertEqual(queued["mission_id"], mission["mission_id"])
        self.bridge.set_state(
            "WAITING_PICKUP",
            message="รอรับอาหาร",
            current_order_index=0,
        )

        self.assert_rejected_confirmation(
            {"mission_id": "old-mission", "order_index": 0},
            409,
        )
        self.assert_rejected_confirmation(
            {"mission_id": mission["mission_id"], "order_index": 1},
            409,
        )

        status, _ = self.post_json(
            "/api/mission/pickup-confirmed",
            {"mission_id": mission["mission_id"], "order_index": 0},
            origin=self.origin,
        )
        self.assertEqual(status, 202)
        self.assertEqual(
            self.bridge.take_pickup_confirmation(timeout=0.1),
            (mission["mission_id"], 0),
        )

        status, _ = self.post_json(
            "/api/mission/pickup-confirmed",
            {"mission_id": mission["mission_id"], "order_index": 0},
            origin=self.origin,
        )
        self.assertEqual(status, 409)

    def test_reset_is_only_accepted_for_latched_error(self):
        status, _ = self.post_json(
            "/api/mission/reset", {}, origin=self.origin
        )
        self.assertEqual(status, 409)

        self.bridge.set_state("ERROR", message="ไปไม่ถึงโต๊ะ", error="ไปไม่ถึงโต๊ะ")
        status, result = self.post_json(
            "/api/mission/reset", {}, origin=self.origin
        )
        self.assertEqual(status, 202)
        self.assertTrue(result["accepted"])
        self.assertTrue(self.bridge.take_reset(timeout=0.1))

    def test_rejects_cross_origin_post(self):
        status, _ = self.post_json(
            "/api/mission/start",
            {"orders": [{"shelf": 1, "table_id": 1, "loaded_confirmed": True}]},
            origin="http://attacker.invalid",
        )
        self.assertEqual(status, 403)

    def test_terminal_client_uses_same_mission_and_pickup_api(self):
        client = PosApi(self.server.url)
        accepted = client.submit(
            [{"shelf": 1, "table_id": 2, "loaded_confirmed": True}]
        )
        queued = self.bridge.take_mission(timeout=0.1)
        self.assertEqual(queued["mission_id"], accepted["mission_id"])

        self.bridge.set_state(
            "WAITING_PICKUP",
            message="รอรับอาหาร",
            current_order_index=0,
        )
        result = client.confirm_pickup(accepted["mission_id"], 0)
        self.assertEqual(result, {"accepted": True})

        with self.assertRaises(PosApiError):
            client.submit(
                [{"shelf": 2, "table_id": 1, "loaded_confirmed": True}]
            )

    def assert_rejected_confirmation(self, payload, expected_status):
        status, _ = self.post_json(
            "/api/mission/pickup-confirmed",
            payload,
            origin=self.origin,
        )
        self.assertEqual(status, expected_status)


if __name__ == "__main__":
    unittest.main()

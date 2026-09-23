import sys
import threading
import time
import unittest
from pathlib import Path
from queue import Empty, Queue

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from delivery_fsm import DeliveryFSM, State  # noqa: E402
from pos_server import PosBridge  # noqa: E402


class FakeMotion:
    def __init__(self):
        self.stopped = 0
        self.continuous_stopped = 0

    def stop(self):
        self.stopped += 1

    def stop_continuous(self):
        self.continuous_stopped += 1
        return True

    def turn(self, _degrees):
        return True

    def forward(self, _distance):
        return True


class FakeOdometry:
    def __init__(self):
        self.reset_count = 0

    @property
    def pose(self):
        return 0.0, 0.0, 0.0

    def reset(self):
        self.reset_count += 1


class FakeShelf:
    def __init__(self):
        self.overrides = Queue()

    def lcd_print(self, _row, _text):
        pass

    def consume_override(self):
        try:
            self.overrides.get_nowait()
            return True
        except Empty:
            return False


class FakeWaypoints:
    def __init__(self, results=None):
        self.calls = []
        self.results = list(results or [])

    def navigate_to(self, x, y, target_theta_deg=None):
        self.calls.append((x, y, target_theta_deg))
        return self.results.pop(0) if self.results else True


class DeliveryFsmPosTests(unittest.TestCase):
    def make_fsm(self, bridge, shelf=None, waypoints=None):
        motion = FakeMotion()
        odometry = FakeOdometry()
        shelf = shelf or FakeShelf()
        waypoints = waypoints or FakeWaypoints()
        fsm = DeliveryFSM(
            motion=motion,
            odometry=odometry,
            shelf=shelf,
            pos_bridge=bridge,
            waypoint_controller=waypoints,
        )
        return fsm, motion, odometry, shelf, waypoints

    def begin_mission(self, bridge, fsm, orders):
        accepted = bridge.submit_mission(orders)
        fsm._state_wait_for_pos()
        self.assertEqual(fsm._state, State.DELIVERING)
        return accepted

    def wait_for_pickup_state(self, bridge, thread):
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if bridge.snapshot()["state"] == "WAITING_PICKUP":
                return
            if not thread.is_alive():
                break
            time.sleep(0.005)
        self.fail("FSM did not enter WAITING_PICKUP")

    def confirm_current_pickup(self, bridge, accepted, fsm):
        thread = threading.Thread(target=fsm._state_wait_pickup)
        thread.start()
        self.wait_for_pickup_state(bridge, thread)
        snapshot = bridge.snapshot()
        bridge.confirm_pickup(accepted["mission_id"], snapshot["current_order_index"])
        thread.join(timeout=1.0)
        self.assertFalse(thread.is_alive())

    def test_single_order_waits_for_manual_pickup_then_returns_home(self):
        bridge = PosBridge()
        fsm, _motion, odometry, _shelf, waypoints = self.make_fsm(bridge)
        accepted = self.begin_mission(
            bridge,
            fsm,
            [{"shelf": 1, "table_id": 1, "loaded_confirmed": True}],
        )

        fsm._state_delivering()
        self.assertEqual(fsm._state, State.WAIT_PICKUP)
        self.assertEqual(bridge.snapshot()["state"], "NAVIGATING")
        self.assertEqual(
            waypoints.calls[:2],
            [(2.0, 0.0, None), (2.0, 0.6, 90.0)],
        )

        wait_thread = threading.Thread(target=fsm._state_wait_pickup)
        wait_thread.start()
        self.wait_for_pickup_state(bridge, wait_thread)
        wait_thread.join(timeout=0.15)
        self.assertTrue(wait_thread.is_alive(), "Pickup must not auto-advance on timeout")

        bridge.confirm_pickup(accepted["mission_id"], 0)
        wait_thread.join(timeout=1.0)
        self.assertFalse(wait_thread.is_alive())
        self.assertEqual(fsm._state, State.CHECK_REMAIN)

        fsm._state_check_remain()
        self.assertEqual(fsm._state, State.RETURN_STATION)
        fsm._state_return_station()
        self.assertEqual(fsm._state, State.WAIT_FOR_POS)
        self.assertEqual(bridge.snapshot()["state"], "COMPLETED")
        self.assertEqual(odometry.reset_count, 1)
        self.assertEqual(
            waypoints.calls[-2:],
            [(2.0, 0.0, None), (0.0, 0.0, 0.0)],
        )

    def test_two_orders_are_sent_by_shelf_order_and_confirmed_one_at_a_time(self):
        bridge = PosBridge()
        fsm, _motion, _odometry, _shelf, waypoints = self.make_fsm(bridge)
        accepted = self.begin_mission(
            bridge,
            fsm,
            [
                {"shelf": 2, "table_id": 2, "loaded_confirmed": True},
                {"shelf": 1, "table_id": 1, "loaded_confirmed": True},
            ],
        )

        self.assertEqual([item.shelf for item in fsm._orders], [1, 2])
        fsm._state_delivering()
        self.confirm_current_pickup(bridge, accepted, fsm)
        fsm._state_check_remain()
        self.assertEqual(fsm._current_order_index, 1)
        self.assertEqual(fsm._state, State.DELIVERING)

        fsm._state_delivering()
        self.assertEqual(waypoints.calls[-1], (2.0, -0.6, -90.0))
        self.confirm_current_pickup(bridge, accepted, fsm)
        fsm._state_check_remain()
        self.assertEqual(fsm._state, State.RETURN_STATION)

    def test_physical_override_completes_pickup_wait(self):
        bridge = PosBridge()
        shelf = FakeShelf()
        fsm, _motion, _odometry, shelf, _waypoints = self.make_fsm(bridge, shelf=shelf)
        self.begin_mission(
            bridge,
            fsm,
            [{"shelf": 1, "table_id": 2, "loaded_confirmed": True}],
        )
        fsm._state_delivering()
        shelf.overrides.put(True)

        wait_thread = threading.Thread(target=fsm._state_wait_pickup)
        wait_thread.start()
        self.wait_for_pickup_state(bridge, wait_thread)
        wait_thread.join(timeout=0.15)
        self.assertTrue(wait_thread.is_alive(), "An override pressed before arrival must be discarded")

        shelf.overrides.put(True)
        wait_thread.join(timeout=1.0)
        self.assertFalse(wait_thread.is_alive())
        self.assertEqual(fsm._state, State.CHECK_REMAIN)

    def test_navigation_failure_latches_error_and_stops_robot(self):
        bridge = PosBridge()
        waypoints = FakeWaypoints(results=[False])
        fsm, motion, _odometry, _shelf, _waypoints = self.make_fsm(
            bridge,
            waypoints=waypoints,
        )
        self.begin_mission(
            bridge,
            fsm,
            [{"shelf": 1, "table_id": 1, "loaded_confirmed": True}],
        )

        fsm._state_delivering()
        self.assertEqual(fsm._state, State.ERROR)
        self.assertEqual(bridge.snapshot()["state"], "ERROR")
        self.assertEqual(motion.stopped, 1)
        self.assertGreaterEqual(motion.continuous_stopped, 1)


if __name__ == "__main__":
    unittest.main()

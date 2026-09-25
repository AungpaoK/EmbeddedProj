import sys
import threading
import time
import unittest
from pathlib import Path
from queue import Empty, Queue

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from delivery_fsm import DeliveryFSM, State
from pos_server import PosBridge


class FakeMotion:
    def __init__(self):
        self.stopped = 0
        self.continuous_stopped = 0
        self.mission_active = []

    def set_delivery_mission_active(self, active):
        self.mission_active.append(bool(active))

    def set_turn_intent(self, _direction):
        pass

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
    def __init__(self, table_results=None, begin_result=True, return_result=True):
        self.calls = []
        self.table_results = list(table_results or [])
        self.begin_result = begin_result
        self.return_result = return_result
        self.last_error = "navigation failed"

    def begin_mission(self):
        self.calls.append(("begin",))
        return self.begin_result

    def go_to_table(self, table_id):
        self.calls.append(("table", table_id))
        return self.table_results.pop(0) if self.table_results else True

    def return_home(self):
        self.calls.append(("home",))
        return self.return_result


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
        self.assertEqual(waypoints.calls[:2], [("begin",), ("table", 1)])

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
        self.assertEqual(_motion.mission_active, [True])
        fsm._state_return_station()
        self.assertEqual(fsm._state, State.WAIT_FOR_POS)
        self.assertEqual(_motion.mission_active, [True, False])
        self.assertEqual(bridge.snapshot()["state"], "COMPLETED")
        self.assertEqual(odometry.reset_count, 1)
        self.assertEqual(waypoints.calls[-1], ("home",))

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
        self.assertEqual(waypoints.calls[-1], ("table", 2))
        self.confirm_current_pickup(bridge, accepted, fsm)
        fsm._state_check_remain()
        self.assertEqual(fsm._state, State.RETURN_STATION)
        self.assertEqual(_motion.mission_active, [True])

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
        waypoints = FakeWaypoints(table_results=[False])
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
        self.assertEqual(motion.mission_active, [True, False])


if __name__ == "__main__":
    unittest.main()

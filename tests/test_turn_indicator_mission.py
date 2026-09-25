import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from motion_client import MotionClient, RosMotionClient
from turn_indicator import display_signal_for_intent


class FakeSerial:
    def __init__(self):
        self.writes = []

    def write(self, payload):
        self.writes.append(payload.decode("utf-8"))

    def flush(self):
        pass


class FakeRosNode:
    def __init__(self):
        self.mission_states = []
        self.turn_intents = []

    def publish_delivery_mission_active(self, active):
        self.mission_states.append(active)

    def publish_turn_intent(self, direction):
        self.turn_intents.append(direction)


class TurnIndicatorMissionTests(unittest.TestCase):
    def test_direction_mapping_accounts_for_rear_matrix_and_steering_inversion(self):
        self.assertEqual(display_signal_for_intent("LEFT"), "RIGHT")
        self.assertEqual(display_signal_for_intent("RIGHT"), "LEFT")
        self.assertEqual(display_signal_for_intent("LEFT", invert_steer=True), "LEFT")
        self.assertEqual(display_signal_for_intent("RIGHT", invert_steer=True), "RIGHT")
        self.assertEqual(display_signal_for_intent("OFF", invert_steer=True), "OFF")

    def test_serial_turn_signal_is_suppressed_outside_delivery_mission(self):
        ser = FakeSerial()
        client = MotionClient(ser)

        with patch.object(client, "_send_and_wait", return_value=True):
            self.assertTrue(client.turn(45.0))

        self.assertEqual(ser.writes, [])

    def test_serial_turn_signal_is_directional_and_stops_after_turn(self):
        ser = FakeSerial()
        client = MotionClient(ser)
        client.set_delivery_mission_active(True)

        with patch.object(client, "_send_and_wait", return_value=True):
            self.assertTrue(client.turn(45.0))

        self.assertEqual(ser.writes, ["INDICATOR:RIGHT\n", "INDICATOR:OFF\n"])

    def test_velocity_adjustments_never_select_a_turn_signal(self):
        ser = FakeSerial()
        client = MotionClient(ser)
        client.set_delivery_mission_active(True)

        self.assertTrue(client.drive_continuous(0.2, 0.4))
        self.assertTrue(client.drive_continuous(0.2, -0.4))

        self.assertEqual(ser.writes, ["V:0.131,0.269\n", "V:0.269,0.131\n"])

    def test_ros_client_suppresses_turn_intent_outside_mission_and_closes_gate(self):
        node = FakeRosNode()
        client = RosMotionClient(node)

        client.set_turn_intent("LEFT")
        client.set_delivery_mission_active(True)
        client.set_turn_intent("LEFT")
        client.set_delivery_mission_active(False)

        self.assertEqual(node.turn_intents, ["OFF", "OFF", "LEFT", "OFF"])
        self.assertEqual(node.mission_states, [True, False])


if __name__ == "__main__":
    unittest.main()

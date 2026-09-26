#!/usr/bin/env python3
"""Exercise both turn-indicator directions through the running ROS bridge.

This script only publishes /delivery_mission_active and /turn_intent. It never
publishes /cmd_vel or opens the Arduino serial port. Run only while the robot
is idle and no delivery mission is active.
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError as exc:
    print(f"ROS 2 Python packages unavailable: {exc}", file=sys.stderr)
    raise SystemExit(2)


class TurnIndicatorTest(Node):
    def __init__(self):
        super().__init__("turn_indicator_test")
        self.mission_pub = self.create_publisher(Bool, "/delivery_mission_active", 10)
        self.intent_pub = self.create_publisher(String, "/turn_intent", 10)

    def publish_mission(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self.mission_pub.publish(message)

    def publish_intent(self, direction: str) -> None:
        message = String()
        message.data = direction
        self.intent_pub.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Blink LEFT then RIGHT using the running slam_bridge; no motor commands are sent."
    )
    parser.add_argument(
        "--confirm-idle",
        action="store_true",
        help="confirm that the robot is idle and no delivery mission is active",
    )
    parser.add_argument("--on-seconds", type=float, default=2.0, help="seconds to show each direction")
    parser.add_argument("--off-seconds", type=float, default=1.0, help="seconds between directions")
    args = parser.parse_args()

    if not args.confirm_idle:
        parser.error("refusing to run without --confirm-idle; make sure no delivery mission is active")
    if args.on_seconds <= 0 or args.off_seconds < 0:
        parser.error("on-seconds must be positive and off-seconds cannot be negative")

    rclpy.init()
    node = TurnIndicatorTest()
    try:
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            if node.mission_pub.get_subscription_count() > 0 and node.intent_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            print("No slam_bridge subscribers found; start the robot bridge first.", file=sys.stderr)
            return 1

        print("Testing turn indicators only; /cmd_vel is not used.")
        node.publish_mission(True)
        time.sleep(0.3)
        for direction in ("LEFT", "OFF", "RIGHT", "OFF"):
            print(f"INDICATOR:{direction}")
            node.publish_intent(direction)
            time.sleep(args.on_seconds if direction != "OFF" else args.off_seconds)
        return 0
    except KeyboardInterrupt:
        print("Test interrupted.")
        return 130
    finally:
        if rclpy.ok():
            node.publish_intent("OFF")
            time.sleep(0.1)
            node.publish_mission(False)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

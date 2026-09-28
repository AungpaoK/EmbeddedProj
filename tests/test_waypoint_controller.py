import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from waypoint_controller import WaypointController, normalize_angle


class SimPose:
    def __init__(self, heading=0.0, x=0.0, y=0.0):
        self.x = x
        self.y = y
        self.heading = heading

    @property
    def value(self):
        return self.x, self.y, self.heading


class SimMotion:
    def __init__(self):
        self.linear = 0.0
        self.angular = 0.0
        self.commands = []
        self.stop_count = 0
        self.turn_intents = []
        self.mission_states = []

    def set_turn_intent(self, direction):
        self.turn_intents.append(direction)

    def set_delivery_mission_active(self, active):
        self.mission_states.append(bool(active))

    def drive_continuous(self, linear, angular, wheel_base=0.343):
        self.linear = linear
        self.angular = angular
        self.commands.append((linear, angular))
        return True

    def stop_continuous(self):
        self.linear = 0.0
        self.angular = 0.0
        self.stop_count += 1
        return True


class SimClock:
    def __init__(self, pose, motion):
        self.now = 0.0
        self.pose = pose
        self.motion = motion

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        midpoint = self.pose.heading + self.motion.angular * seconds / 2.0
        self.pose.x += self.motion.linear * math.cos(midpoint) * seconds
        self.pose.y += self.motion.linear * math.sin(midpoint) * seconds
        self.pose.heading = normalize_angle(
            self.pose.heading + self.motion.angular * seconds
        )
        self.now += seconds


class ClearSafety:
    is_obstacle_detected = False


class DelayedClearSafety:
    def __init__(self, blocked_checks):
        self.blocked_checks = blocked_checks

    @property
    def is_obstacle_detected(self):
        if self.blocked_checks <= 0:
            return False
        self.blocked_checks -= 1
        return True


class OneShotObstacleSafety:
    """Report one obstacle after the simulated robot has moved a little."""

    def __init__(self, pose, trigger_x=0.05):
        self.pose = pose
        self.trigger_x = trigger_x
        self.reported = False

    @property
    def is_obstacle_detected(self):
        if self.reported or self.pose.x < self.trigger_x:
            return False
        self.reported = True
        return True


def make_controller(*, heading=0.0, x=0.0, y=0.0, safety=None, **controller_kwargs):
    pose = SimPose(heading, x=x, y=y)
    motion = SimMotion()
    clock = SimClock(pose, motion)
    controller = WaypointController(
        motion,
        safety or ClearSafety(),
        lambda: pose.value,
        ready_provider=lambda: True,
        pose_fresh_provider=lambda: True,
        **controller_kwargs,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    return controller, pose, motion, clock


class WaypointControllerTests(unittest.TestCase):
    def test_turn_crosses_pi_boundary_in_error_reducing_direction(self):
        controller, pose, motion, _clock = make_controller(
            heading=math.radians(179.0)
        )
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.turn_to_heading(math.radians(-170.0)))

        first_turn = next(angular for linear, angular in motion.commands if not linear)
        self.assertGreater(first_turn, 0.0)
        self.assertEqual(motion.turn_intents, ["LEFT", "OFF"])
        self.assertLess(
            abs(normalize_angle(pose.heading - math.radians(-170.0))),
            math.radians(2.5),
        )

    def test_arbitrary_table_order_returns_home_facing_start_heading(self):
        start_heading = math.radians(20.0)
        controller, pose, _motion, _clock = make_controller(heading=start_heading)
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.go_to_table(2))
        self.assertTrue(controller.go_to_table(1))
        self.assertTrue(controller.return_home())

        self.assertEqual(controller._location, "home")
        self.assertLess(
            abs(normalize_angle(pose.heading - start_heading)),
            math.radians(2.5),
        )
        self.assertLess(math.hypot(pose.x, pose.y), 0.20)

    def test_same_table_second_order_does_not_move_again(self):
        controller, _pose, motion, _clock = make_controller()
        self.assertTrue(controller.begin_mission())
        self.assertTrue(controller.go_to_table(1))
        command_count = len(motion.commands)

        self.assertTrue(controller.go_to_table(1))

        self.assertEqual(len(motion.commands), command_count)

    def test_obstacle_pauses_without_consuming_drive_timeout(self):
        safety = DelayedClearSafety(blocked_checks=10)
        controller, _pose, motion, clock = make_controller(safety=safety)
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.drive_forward(0.20, 0.0, timeout_s=2.0))

        self.assertGreaterEqual(motion.stop_count, 10)
        self.assertGreater(clock.now, 1.0)

    def test_table_leg_clears_obstacle_indicator_after_reaching_parking_pose(self):
        pose = SimPose()
        motion = SimMotion()
        clock = SimClock(pose, motion)
        safety = OneShotObstacleSafety(pose)
        obstacle_events = []
        controller = WaypointController(
            motion,
            safety,
            lambda: pose.value,
            ready_provider=lambda: True,
            pose_fresh_provider=lambda: True,
            obstacle_handler=obstacle_events.append,
            monotonic=clock.monotonic,
            sleep=clock.sleep,
        )
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.drive_forward(0.20, 0.0, table_id=1))

        self.assertGreaterEqual(motion.stop_count, 2)
        self.assertEqual(obstacle_events, [True, False])
        self.assertGreaterEqual(pose.x, 0.10)

    def test_obstacle_before_table_parking_pose_still_pauses(self):
        safety = DelayedClearSafety(blocked_checks=3)
        controller, _pose, motion, _clock = make_controller(safety=safety)
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.drive_forward(0.20, 0.0, table_id=1))

        # The table-specific arrival tolerance must not allow the controller
        # to drive through an obstacle encountered before the parking pose.
        self.assertGreaterEqual(motion.stop_count, 3)

    def test_straight_heading_corrections_do_not_set_turn_intent(self):
        controller, _pose, motion, _clock = make_controller(
            heading=math.radians(-4.0)
        )
        self.assertTrue(controller.begin_mission())

        self.assertTrue(controller.drive_forward(0.10, 0.0))

        self.assertEqual(motion.turn_intents, [])

    def test_cross_track_correction_turns_back_toward_line(self):
        controller, pose, motion, _clock = make_controller()
        self.assertTrue(controller.begin_mission())

        # Simulate a lateral odometry deviation after the mission frame was
        # captured. Positive y is left of a +x route, so correction must turn
        # right (negative angular velocity).
        pose.y = 0.12
        self.assertTrue(
                controller.drive_to_waypoint(
                0.80,
                0.0,
                0.0,
                line_start=(0.0, 0.0),
                timeout_s=8.0,
            )
        )

        self.assertLess(pose.y, 0.12)
        self.assertTrue(any(linear > 0.0 and angular < 0.0 for linear, angular in motion.commands))

    def test_cross_track_can_be_disabled_for_legacy_ab_comparison(self):
        controller, _pose, _motion, _clock = make_controller(cross_track_enabled=False)
        self.assertTrue(controller.begin_mission())

        self.assertAlmostEqual(
            controller._compute_drive_correction(0.0, 0.20),
            0.0,
            places=6,
        )

    def test_waypoint_requires_heading_after_reaching_position(self):
        controller, pose, motion, _clock = make_controller()
        self.assertTrue(controller.begin_mission())
        pose.x = 0.15
        pose.heading = math.radians(90.0)

        self.assertTrue(
            controller.drive_to_waypoint(
                0.15,
                0.0,
                0.0,
                line_start=(0.0, 0.0),
                timeout_s=5.0,
            )
        )

        self.assertLess(abs(pose.heading), math.radians(5.0))
        self.assertTrue(any(linear == 0.0 for linear, _angular in motion.commands))

    def test_cancelled_heading_turn_clears_turn_intent(self):
        controller, _pose, motion, _clock = make_controller()
        self.assertTrue(controller.begin_mission())
        controller._active = False

        self.assertFalse(controller.turn_to_heading(math.radians(90.0)))

        self.assertEqual(motion.turn_intents, ["LEFT", "OFF"])

    def test_cancel_closes_delivery_indicator_gate(self):
        controller, _pose, motion, _clock = make_controller()
        self.assertTrue(controller.begin_mission())

        controller.cancel()

        self.assertEqual(motion.turn_intents, ["OFF"])
        self.assertEqual(motion.mission_states, [False])


if __name__ == "__main__":
    unittest.main()

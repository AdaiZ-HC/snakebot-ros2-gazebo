"""Check the long-body safety envelope and emergency state transitions."""

import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import Mock

import rclpy

spec = importlib.util.spec_from_file_location(
    'snake_roam', Path(__file__).resolve().parents[1] / 'scripts' / 'snake_roam.py')
roam = importlib.util.module_from_spec(spec)
spec.loader.exec_module(roam)


class RoamSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = roam.SnakeRoam()
        self.node.joints = {f'joint_{i}': 0.0 for i in range(1, 9)}

    def tearDown(self):
        self.node.destroy_node()

    def test_emergency_overrides_random_choice(self):
        self.node.pose = (0.95, 0.0, 0.0)
        v, w, distance = self.node.choose_velocity(10.0)
        self.assertLessEqual(distance, 0.4)
        self.assertEqual(self.node.state, 'DODGE')
        self.assertEqual(v, 0.0)
        self.assertAlmostEqual(abs(w), 1.0)

    def test_circle_stays_bounded_and_expires(self):
        self.node.pose = (0.45, 0.0, 0.0)
        self.node.rng = Mock()
        self.node.rng.random.return_value = 0.1
        self.node.rng.choice.return_value = 1
        self.node.rng.uniform.side_effect = lambda a, b: (a + b) / 2
        v, w, _ = self.node.choose_velocity(10.0)
        self.assertEqual(self.node.state, 'CIRCLE')
        self.assertTrue(13 <= self.node.until <= 15)
        self.assertTrue(0 <= v <= 0.2 and abs(w) <= 0.3)
        self.node.choose_velocity(15.1)
        self.assertEqual(self.node.state, 'ROAM')

    def test_tail_collision_detected_when_head_is_clear(self):
        self.node.pose = (2.4, 0.0, 0.0)
        head = self.node.world_points()[1]
        self.assertGreater(math.hypot(head[0] - 1.5, head[1]), 1.0)
        self.assertLess(self.node.clearance(), 0.0)

    def test_missing_odometry_stops(self):
        self.node.v, self.node.w = 0.2, 0.4
        self.node.tick()
        self.assertEqual((self.node.v, self.node.w), (0.0, 0.0))

    def test_stuck_uses_actual_motion_not_dead_reckoning(self):
        self.node.history.extend([(0.0, 0.0, 0.0, 0.0), (1.0, 0.01, 0.0, 0.1),
                                  (2.0, 0.01, 0.0, 0.2), (3.0, 0.02, 0.0, 0.3)])
        self.assertTrue(self.node.is_stuck())
        self.node.history.clear()
        self.node.history.extend([(0.0, 0.0, 0.0, 0.0), (3.0, 0.3, 0.0, 0.3)])
        self.assertFalse(self.node.is_stuck())

    def test_stationary_command_does_not_trigger_recovery(self):
        self.node.history.extend([(0.0, 0.0, 0.0, 0.0), (3.0, 0.0, 0.0, 0.0)])
        self.assertFalse(self.node.is_stuck())

    def test_recovery_reverses_for_two_seconds_then_roams(self):
        self.node.pose = (0.0, 0.0, 0.0)
        self.node.start_recovery(10.0, 'test blocked motion')
        v, w, _ = self.node.choose_velocity(11.0)
        self.assertEqual(self.node.state, 'DODGE')
        self.assertEqual(v, -0.1)
        self.assertAlmostEqual(abs(w), 1.0)
        self.node.choose_velocity(12.1)
        self.assertEqual(self.node.state, 'ROAM')
        self.assertFalse(self.node.recovering)

    def test_ease_reaches_target_without_a_step(self):
        value = rate = 0.0
        samples = []
        for _ in range(10):
            value, rate = roam.ease_velocity(value, rate, 0.2, 0.05)
            samples.append(value)
        self.assertLess(samples[0], 0.02)
        self.assertGreater(samples[-1], 0.19)
        self.assertLess(samples[-1], 0.2)
        self.assertEqual(samples, sorted(samples))


if __name__ == '__main__':
    unittest.main()

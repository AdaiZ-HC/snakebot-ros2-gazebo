"""Regression checks for the rejection that froze all ten controller joints."""

from pathlib import Path
import sys
import unittest

import yaml
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from snake_behavior import ALL_JOINTS, make_trajectory
from snake_behavior import SnakeBehavior
from behavior_core import Biology
from types import SimpleNamespace


class TrajectoryContractTest(unittest.TestCase):
    def test_special_targets_and_continuous_frequency_phase(self):
        node = object.__new__(SnakeBehavior)
        node.bio = Biology(42)
        node.bio.update(0.0)
        node.phase, node.phase_time = 1.2, 0.0
        node.nav = SimpleNamespace(pose=None)
        node.bio.enter('COILED', 0.0)
        positions, velocities, _, _ = node.body_targets(1.1)
        self.assertTrue(all(abs(q-0.65)<1e-9 for q in positions))
        self.assertTrue(all(abs(v)<1e-9 for v in velocities))
        node.phase = node.phase_at(2.0)
        node.phase_time = 2.0
        before = node.phase_at(2.0)
        node.bio.enter('WRAP', 2.0)
        self.assertAlmostEqual(node.phase_at(2.0), before)
        self.assertAlmostEqual(node.phase_at(3.0)-before, 2*3.141592653589793*0.85)
        positions, velocities, values, _ = node.body_targets(3.1)
        self.assertAlmostEqual(values['amplitude'], 0.6)
        self.assertAlmostEqual(values['frequency'], 1.0)
        self.assertTrue(all(abs(q)<=0.65+1e-9 for q in positions))

    def test_streamed_message_and_controller_agree(self):
        config = yaml.safe_load((PACKAGE/'config/controllers.yaml').read_text())
        params = config['joint_trajectory_controller']['ros__parameters']
        message = make_trajectory(Time(seconds=5), [0.0]*10, [0.5]*10, 0.06)
        wire = deserialize_message(serialize_message(message), JointTrajectory)
        self.assertEqual(wire.joint_names, [f'joint_{i}' for i in range(1, 9)]
                         + ['head_pitch_joint', 'tongue_joint'])
        self.assertEqual(wire.joint_names, params['joints'])
        self.assertEqual(len(wire.points), 1)
        self.assertEqual(len(wire.points[0].positions), 10)
        self.assertEqual(len(wire.points[0].velocities), 10)
        # This exact combination was rejected before the configuration fix.
        self.assertTrue(any(wire.points[0].velocities))
        self.assertTrue(params['allow_nonzero_velocity_at_trajectory_end'])
        self.assertTrue(params['interpolate_from_desired_state'])
        self.assertGreater(wire.points[0].time_from_start.nanosec, 0)

    def test_incomplete_and_nonfinite_commands_fail_before_publishing(self):
        for positions, velocities in (([0.0]*8, [0.0]*10),
                                      ([0.0]*10, [0.0]*9),
                                      ([float('nan')]+[0.0]*9, [0.0]*10)):
            with self.subTest(positions=len(positions), velocities=len(velocities)):
                with self.assertRaises(ValueError):
                    make_trajectory(Time(seconds=5), positions, velocities, 0.06)


if __name__ == '__main__':
    unittest.main()

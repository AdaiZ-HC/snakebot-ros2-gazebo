"""Behavior timing, non-sinusoidal flicks, and safety regression checks."""

import importlib.util
import math
from pathlib import Path
import random
import unittest

script = Path(__file__).resolve().parents[1]/'scripts/behavior_core.py'
spec = importlib.util.spec_from_file_location('behavior_core', script)
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)


class BiologyTest(unittest.TestCase):
    def test_flick_timing_and_count_across_random_seeds(self):
        for seed in range(100):
            train = core.FlickTrain(random.Random(seed))
            self.assertTrue(3 <= train.count <= 5)
            self.assertTrue(all(0.1-1e-9 <= p <= 0.3+1e-9 for p in train.pauses))
            self.assertLessEqual(train.pulses[-1][0]+0.25, 1.95+1e-9)
            for start, length in train.pulses:
                self.assertTrue(0.06 <= length <= 0.12)
                self.assertAlmostEqual(train.sample(start+0.05)[0], length/2)
                self.assertEqual(train.sample(start+0.12)[2], 'HOLD')
                self.assertLess(train.sample(start+0.20)[1], 0)
                self.assertEqual(train.sample(start+0.27)[0], 0)

    def test_one_second_blend_has_zero_endpoint_rates(self):
        blend = core.Blend({'head': 0.0}, {'head': 0.8}, 10.0)
        self.assertEqual(blend.sample(10.0), ({'head': 0.0}, {'head': 0.0}))
        self.assertAlmostEqual(blend.sample(10.5)[0]['head'], 0.4)
        self.assertEqual(blend.sample(11.0), ({'head': 0.8}, {'head': 0.0}))

    def test_rest_ten_percent_and_conditional_coiled_thirty_percent(self):
        biology = core.Biology(7)
        choices = [biology.choose() for _ in range(20000)]
        rest = choices.count('REST')+choices.count('COILED')
        self.assertTrue(1800 < rest < 2200)
        self.assertTrue(0.25 < choices.count('COILED')/rest < 0.35)

    def test_alert_expires_at_two_seconds_and_retracts(self):
        biology = core.Biology(7)
        biology.update(0.0)
        biology.enter('ALERT', 0.0)
        values, _, _, _, _ = biology.update(1.0)
        self.assertAlmostEqual(values['head'], 0.8)
        self.assertEqual(biology.update(2.0)[2], 0.0)
        self.assertEqual(biology.state, 'ROAM')

    def test_low_crawl_ends_after_two_seconds(self):
        biology = core.Biology(7)
        biology.update(0.0)
        biology.enter('LOW_CRAWL', 0.0)
        values = biology.update(1.0)[0]
        self.assertAlmostEqual(values['head'], -0.2)
        self.assertAlmostEqual(values['speed'], 0.5)
        biology.update(2.0)
        self.assertEqual(biology.state, 'ROAM')

    def test_default_demo_includes_coiled_and_returns_to_roam(self):
        biology = core.Biology()
        states = set()
        for i in range(3001):
            biology.update(i/100)
            states.add(biology.state)
        self.assertTrue({'COILED', 'ROAM'} <= states)

    def test_roam_start_and_visible_wave_in_every_state(self):
        biology = core.Biology(7)
        biology.update(0.0)
        self.assertEqual(biology.state, 'ROAM')
        self.assertAlmostEqual(biology.update(1.0)[0]['amplitude'], 0.4)
        for state in ('REST', 'ALERT', 'LOW_CRAWL'):
            biology.enter(state, 1.0)
            amplitude = biology.update(2.0)[0]['amplitude']
            self.assertGreater(amplitude, 0.0)
            if state != 'REST':
                self.assertGreater(amplitude, 0.3)

    def test_rest_expires_without_an_external_event(self):
        biology = core.Biology(7)
        biology.update(0.0)
        biology.enter('REST', 0.0)
        biology.update(5.0)
        self.assertEqual(biology.state, 'ROAM')

    def test_actual_blocked_motion_triggers_reverse(self):
        nav = core.Navigation([{'x': 10, 'y': 10, 'radius': 0.2}])
        nav.pose = (0, 0, 0)
        nav.joints = {name: 0 for name in core.ALL_JOINTS}
        for i in range(63):
            nav.command_v = 0.1
            command = nav.step(i*0.05, 0.05, 1.0)
        self.assertTrue(nav.recovering)
        self.assertEqual(command[0], -0.15)
        self.assertEqual(abs(command[1]), 1.5)

    def test_intentional_rest_does_not_trigger_recovery(self):
        nav = core.Navigation([{'x': 10, 'y': 10, 'radius': 0.2}])
        nav.pose = (0, 0, 0)
        nav.joints = {name: 0 for name in core.ALL_JOINTS}
        for i in range(100):
            nav.command_v = 0.0
            nav.step(i*0.05, 0.05, 0.0)
        self.assertFalse(nav.recovering)

    def test_coiled_and_wrap_are_not_interrupted_by_random_choice(self):
        for state in ('COILED', 'WRAP'):
            biology = core.Biology(42)
            biology.update(0.0)
            biology.enter(state, 0.0)
            biology.next_choice = 0.1
            self.assertEqual(biology.transition_duration, 1.0)
            biology.update(biology.until-0.01)
            self.assertEqual(biology.state, state)
            end = biology.until
            biology.update(end+0.001)
            self.assertEqual(biology.state, 'ROAM')
            self.assertEqual(biology.transition_duration, 1.0)

    def test_short_and_special_blends_respect_endpoint_rates(self):
        blend = core.Blend({'a': 0.0}, {'a': 1.0}, 0.0, 0.3)
        self.assertEqual(blend.sample(0.3), ({'a': 1.0}, {'a': 0.0}))
        self.assertAlmostEqual(blend.sample(0.15)[0]['a'], 0.5)

    def test_wrap_orbit_and_exit_keep_safe_radius(self):
        nav = core.Navigation([{'x': 0, 'y': 0, 'radius': 0.25}], seed=42)
        nav.pose = (-0.95, 0.0, 0.0)
        nav.joints = {name: 0.0 for name in core.ALL_JOINTS}
        entered = exited = None
        for i in range(1600):
            t = i*0.02
            v, w = nav.step(t, 0.02, 1.0)
            nav.command_v = v
            x, y, a = nav.pose
            nav.pose = (x+v*math.cos(a)*0.02, y+v*math.sin(a)*0.02, a+w*0.02)
            if nav.state=='WRAP':
                entered = t if entered is None else entered
                self.assertGreaterEqual(math.hypot(nav.pose[0], nav.pose[1]), 0.5)
                self.assertLessEqual(v, 0.35)
            if entered is not None and nav.wrap_completed:
                exited = t
                break
        self.assertIsNotNone(entered)
        self.assertIsNotNone(exited)
        self.assertAlmostEqual(exited-entered, 4.0, places=1)
        self.assertEqual(nav.wrap_aborted, 0)


if __name__ == '__main__':
    unittest.main()

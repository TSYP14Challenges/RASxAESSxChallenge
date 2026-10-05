#!/usr/bin/env python3
"""Regression checks for Gazebo clock/pose handling, using a fake transport."""
from contextlib import redirect_stdout
import csv
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

import drive_step1
import launch_step1
from step1_feedback import Feedback

CONFIG = {'model_name': 'aisle_robot', 'world_name': 'factory'}


def stamp(value):
    return Obj(sec=int(value), nsec=round((value-int(value))*1e9))


def pose_message(x=0.0, y=-14.0, inner=0.0, outer=0.0, name='aisle_robot'):
    pose = Obj(name=name, header=Obj(stamp=stamp(inner)),
               position=Obj(x=x, y=y, z=.005),
               orientation=Obj(x=0.0, y=0.0, z=math.sin(math.pi/4), w=math.cos(math.pi/4)))
    return Obj(header=Obj(stamp=stamp(outer)), pose=[pose])


class FeedbackChecks(unittest.TestCase):
    def setUp(self):
        self.feedback = Feedback(CONFIG)

    def clock(self, value):
        self.feedback.receive_clock(Obj(sim=stamp(value)))

    def test_individual_pose_stamp_with_empty_vector_header(self):
        self.clock(5)
        self.feedback.receive(pose_message(inner=5), self.feedback.topics[0])
        state = self.feedback.state()
        self.assertEqual(state['pose_time'], 5)
        self.assertEqual(state['sim_time'], 5)
        self.assertTrue(state['pose_fresh'])

    def test_zero_pose_stamps_do_not_freeze_simulation_time(self):
        for sim in (1, 2, 3, 4, 5):
            self.clock(sim)
            self.feedback.receive(pose_message(), self.feedback.topics[-1])
            self.assertEqual(self.feedback.state()['sim_time'], sim)
            self.assertTrue(self.feedback.state()['pose_fresh'])

    def test_recent_robot_stream_overrides_scene_snapshot(self):
        self.clock(5)
        self.feedback.receive(pose_message(y=-13, inner=5), self.feedback.topics[0])
        self.feedback.receive(pose_message(y=-14), self.feedback.topics[-1])
        self.assertEqual(self.feedback.state()['y'], -13)
        # Out-of-order poses on the same source cannot rewind the robot.
        self.feedback.receive(pose_message(y=-14, inner=4), self.feedback.topics[0])
        self.assertEqual(self.feedback.state()['y'], -13)

    def test_stale_pose_is_flagged_and_recent_fallback_recovers(self):
        self.clock(1)
        self.feedback.receive(pose_message(inner=1), self.feedback.topics[0])
        self.clock(2)
        self.assertFalse(self.feedback.state()['pose_fresh'])
        self.feedback.receive(pose_message(y=-13, inner=2), self.feedback.topics[2])
        self.assertTrue(self.feedback.state()['pose_fresh'])
        self.assertEqual(self.feedback.state()['y'], -13)

    def test_step2_uses_newer_world_pose_even_while_old_model_pose_is_fresh(self):
        self.feedback = Feedback({**CONFIG, 'pose_max_age': .5, 'prefer_newest_pose': True})
        self.clock(5.2)
        self.feedback.receive(pose_message(y=-11, inner=5), self.feedback.topics[0])
        self.feedback.receive(pose_message(y=-10.8, inner=5.2), self.feedback.topics[2])
        self.assertTrue(self.feedback.state()['pose_fresh'])
        self.assertEqual(self.feedback.state()['y'], -10.8)
        self.feedback.receive(pose_message(y=-11), self.feedback.topics[-1])
        self.assertEqual(self.feedback.state()['y'], -10.8)

    def test_missing_clock_or_wrong_entity_cannot_start_controller(self):
        self.feedback.receive(pose_message(), self.feedback.topics[0])
        self.assertIsNone(self.feedback.state())
        self.feedback = Feedback(CONFIG)
        self.clock(1)
        self.feedback.receive(pose_message(name='aisle_robot::left_wheel'), self.feedback.topics[0])
        self.assertIsNone(self.feedback.state())

    def test_clock_reset_is_detected_even_if_clock_advances_again(self):
        self.clock(5)
        self.feedback.receive(pose_message(), self.feedback.topics[0])
        self.clock(0)
        self.clock(6)
        self.assertTrue(self.feedback.state()['time_reset'])

    def test_high_speed_config_rejects_pose_feedback_older_than_80ms(self):
        self.feedback = Feedback({**CONFIG, 'pose_max_age': .08})
        self.clock(5)
        self.feedback.receive(pose_message(inner=5), self.feedback.topics[0])
        self.assertTrue(self.feedback.state()['pose_fresh'])
        self.clock(5.1)
        self.assertFalse(self.feedback.state()['pose_fresh'])


class FakeTwist:
    def __init__(self):
        self.linear, self.angular = Obj(x=0.0), Obj(z=0.0)


class StartupChecks(unittest.TestCase):
    def run_controller(self, advancing=True, stale=False):
        """Exercise the real main loop and RobotConnection without Gazebo installed."""
        timer = Obj(wall=0.0, sim=0.0, ticks=0, interrupted=False)
        output, commands, subscriptions = io.StringIO(), [], {}

        class Publisher:
            def publish(self, message):
                commands.append((timer.sim, message.linear.x, message.angular.z))
                return True

        class Node:
            def advertise(self, topic, message_type):
                if topic != '/step1/cmd_vel':
                    raise AssertionError(topic)
                return Publisher()

            def subscribe(self, message_type, topic, callback):
                subscriptions[topic] = callback
                return True

        def sleep(seconds):
            timer.wall += max(seconds, .01)
            timer.ticks += 1
            if advancing:
                timer.sim = round(timer.sim+.05, 8)
            subscriptions['/world/factory/clock'](Obj(sim=stamp(timer.sim)))
            # Reproduce the user's world-pose fallback with both stamps unset.
            if not stale or timer.ticks == 1:
                subscriptions['/world/factory/pose/info'](pose_message())
            limit = timer.wall > 11.5 if stale else \
                (timer.sim > 5.5 if advancing else timer.wall > 1.5)
            if limit and not timer.interrupted:
                timer.interrupted = True
                raise KeyboardInterrupt

        with tempfile.TemporaryDirectory(prefix='step1_feedback_') as directory:
            package = Path(directory)
            for relative in ('step1/config.json', 'step1/route.json',
                             'step1/map.json', 'worlds/warehouse.sdf'):
                destination = package/relative
                destination.parent.mkdir(exist_ok=True)
                shutil.copyfile(drive_step1.PACKAGE/relative, destination)
            with patch.object(drive_step1, 'PACKAGE', package), \
                    patch.object(drive_step1, 'transport_imports', return_value=(Node, object, FakeTwist, object)), \
                    patch('sys.argv', ['drive_step1.py']), \
                    patch('time.monotonic', side_effect=lambda: timer.wall), \
                    patch('time.sleep', side_effect=sleep), redirect_stdout(output):
                error = None
                try:
                    result = drive_step1.main()
                except RuntimeError as exception:
                    result, error = None, str(exception)
            report = json.loads((package/'logs/step1_result.json').read_text())
            with (package/'logs/step1_trace.csv').open() as stream:
                trace = list(csv.DictReader(stream))
        self.assertIn('/world/factory/clock', subscriptions)
        self.assertTrue(all(v == w == 0 for _, v, w in commands[-12:]))
        return commands, output.getvalue(), report, trace, result, error

    def test_real_controller_starts_after_delay_with_zero_pose_stamps(self):
        commands, output, report, trace, result, error = self.run_controller()
        moving = [(sim, v, w) for sim, v, w in commands if v or w]
        self.assertTrue(moving, 'The controller never sent a movement command')
        first_sim = report['start_sim_time']
        self.assertGreaterEqual(moving[0][0]-first_sim, .5-1e-6)
        self.assertLess(moving[0][0]-first_sim, .6)
        self.assertIn('Entering the factory', output)
        self.assertEqual(result, 130)
        self.assertIsNone(error)
        self.assertFalse(report['completed'])

    def test_paused_clock_does_not_use_pose_receipt_time_as_sim_time(self):
        commands, output, _, _, _, _ = self.run_controller(advancing=False)
        self.assertTrue(all(v == w == 0 for _, v, w in commands))
        self.assertIn('clock is not advancing', output)

    def test_stale_pose_stops_wheels_and_exits_with_diagnostic(self):
        commands, output, report, _, _, error = self.run_controller(stale=True)
        self.assertTrue(all(v == w == 0 for _, v, w in commands))
        self.assertIn('pose is stale', output)
        self.assertIn('No recent robot pose', error)
        self.assertEqual(report['reason'], error)


class DemoChecks(unittest.TestCase):
    def test_speed_option_accepts_20_metres_per_second(self):
        import argparse
        from step1_settings import load_config, motion_arguments, validate_motion
        config = load_config(drive_step1.PACKAGE)
        parser = argparse.ArgumentParser()
        motion_arguments(parser, config)
        args = parser.parse_args(['--speed', '20'])
        validate_motion(args)
        self.assertEqual(args.speed, 20)
        self.assertEqual(config['linear_speed'], 20)
        with self.assertRaises(ValueError):
            validate_motion(parser.parse_args(['--speed', '20.1']))

    def test_launcher_and_controller_share_the_default_delay(self):
        import argparse
        from step1_settings import load_config, motion_arguments
        config = load_config(drive_step1.PACKAGE)
        parser = argparse.ArgumentParser()
        motion_arguments(parser, config)
        self.assertEqual(parser.parse_args([]).start_delay, config['start_delay'])
        self.assertEqual(config['start_delay'], .5)

    def test_runtime_speed_change_preserves_scene_and_physics_step(self):
        import xml.etree.ElementTree as ET
        source = drive_step1.PACKAGE/'worlds/warehouse_step1_lite.sdf'
        original = ET.parse(source).getroot()
        with tempfile.TemporaryDirectory(prefix='step1_rate_') as directory:
            target = Path(directory)/'runtime.sdf'
            launch_step1.runtime_world(source, target, 6)
            changed = ET.parse(target).getroot()
        self.assertEqual(changed.findtext('world/physics/real_time_factor'), '6')
        self.assertEqual(changed.findtext('world/physics/max_step_size'),
                         original.findtext('world/physics/max_step_size'))
        for tag in ('model', 'plugin', 'gui'):
            self.assertEqual([ET.tostring(e) for e in changed.find('world').findall(tag)],
                             [ET.tostring(e) for e in original.find('world').findall(tag)])


class MotionChecks(unittest.TestCase):
    def test_20_metre_per_second_cruise_and_braking_on_a_long_straight(self):
        from step1_control import Follower
        from step1_settings import load_config
        config = load_config(drive_step1.PACKAGE)
        config['start'] = [0.0, 0.0, 0.0]
        grid = Obj(safe=lambda x, y: True)
        follower = Follower([{'x': 0.0, 'y': 0.0}, {'x': 200.0, 'y': 0.0}], config, grid)
        x, velocity, limited, maximum = 0.0, 0.0, 0.0, 0.0
        dt = .01
        for _ in range(10000):
            command, angular = follower.command(x, 0.0, 0.0, dt)
            if follower.index == 2:
                break
            self.assertEqual(angular, 0.0)
            limited = max(limited-config['linear_deceleration']*dt,
                          min(command, limited+config['linear_acceleration']*dt))
            velocity += (limited-velocity)*(1-math.exp(-dt/.06))
            x += velocity*dt
            maximum = max(maximum, velocity)
            self.assertLessEqual(velocity, 20.0)
        else:
            self.fail('The straight-route test did not finish')
        self.assertAlmostEqual(maximum, 20.0, places=3)
        self.assertLess(abs(200.0-x), config['waypoint_tolerance'])
        self.assertLess(velocity, .10)


if __name__ == '__main__':
    unittest.main(verbosity=1)

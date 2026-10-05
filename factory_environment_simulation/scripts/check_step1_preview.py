#!/usr/bin/env python3
"""Check the complete timed route and exercise its real Gazebo client with fake transport."""
import argparse
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import animate_step1
from check_step1 import clearance
from step1_control import wrap
from step1_map import GridMap, read_geometry
from step1_timeline import Timeline, model_poses

PACKAGE = Path(__file__).resolve().parents[1]


def structural_check():
    config = json.loads((PACKAGE/'step1/config.json').read_text())
    route = json.loads((PACKAGE/'step1/route.json').read_text())
    grid = GridMap.load(PACKAGE/'step1/map.json')
    source = ET.parse(PACKAGE/'worlds/warehouse_lite.sdf').getroot().find('world')
    target = ET.parse(PACKAGE/'worlds/warehouse_step1_preview.sdf').getroot().find('world')
    names = {config['model_name'], config['model_name']+'_left_wheel',
             config['model_name']+'_right_wheel'}
    robots = [m for m in target.findall('model') if m.get('name') in names]
    assert {m.get('name') for m in robots} == names
    environment = [m for m in target.findall('model') if m.get('name') not in names]
    assert [ET.tostring(m) for m in environment] == [ET.tostring(m) for m in source.findall('model')], \
        'Animated world changed environment geometry'
    assert read_geometry(PACKAGE/'worlds/warehouse_step1_preview.sdf', config) == \
        read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    assert target.findtext('physics/max_step_size') == str(config['preview_physics_step'])
    assert any(p.get('name').endswith('::UserCommands') for p in target.findall('plugin'))
    assert not target.findall('include')
    for model in robots:
        assert model.findtext('static') == 'false'
        assert model.findtext('allow_auto_disable') == 'false'
        assert not model.findall('joint')
        assert not model.findall('link/collision')
        for link in model.findall('link'):
            assert link.findtext('gravity') == 'false'
            assert float(link.findtext('inertial/mass')) > 0
        assert not any(p.get('name').endswith('::DiffDrive') for p in model.findall('plugin'))
    print('PASS: animation world keeps the environment, fires, victims and both pits unchanged.')
    return config, route, grid


def timeline_check(config, route, grid):
    timeline = Timeline(route['waypoints'], config, config['preview_duration_seconds'])
    _, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    elapsed, samples, minimum = 0.0, 0, float('inf')
    previous = timeline.sample(0.0)
    for _ in range(20000):
        following = timeline.advance(elapsed, 1/config['preview_frame_rate'])
        current = timeline.sample(following)
        a, b = (previous['x'], previous['y']), (current['x'], current['y'])
        assert math.dist(a, b) <= .25+1e-8
        assert abs(current['yaw']-previous['yaw']) <= .20+1e-8
        assert grid.line_safe(a, b), 'Animated frame cuts across an obstacle or a pit'
        minimum = min(minimum, clearance(*b, obstacles, fires, config))
        for pose in model_poses(current, config['model_name']):
            assert abs(sum(q*q for q in pose[4:])-1) < 1e-8
        elapsed, previous, samples = following, current, samples+1
        if current['mode'] == 'finished':
            break
    else:
        raise AssertionError('Timed tour did not finish')
    assert minimum >= config['safety_margin']
    for arrival, index, _, _ in timeline.arrivals:
        state = timeline.sample(arrival)
        point = route['waypoints'][index]
        assert math.dist((state['x'], state['y']), (point['x'], point['y'])) < 1e-7
    visited = [name for _, _, kind, name in timeline.arrivals if kind == 'aisle']
    expected = [s['name'] for s in route['aisle_segments']]
    assert len(visited) == len(set(visited)) == len(expected) == 15
    assert set(visited) == set(expected)
    assert math.dist((previous['x'], previous['y']), config['start'][:2]) < 1e-7
    assert abs(wrap(previous['yaw']-config['start'][2]-math.pi)) < 1e-7
    print(f'PASS: timed {timeline.duration:g}-second route, all 15 sections, '
          f'{samples} bounded frames, {timeline.linear_speed:.2f} m/s translation, return and stop.')
    return {'duration_seconds': round(timeline.duration, 3), 'translation_speed': timeline.linear_speed,
            'minimum_clearance_metres': round(minimum, 4), 'frames_checked': samples}


def stamp(value):
    return Obj(sec=int(value), nsec=int((value-int(value))*1e9))


class FakePose:
    def __init__(self):
        self.name = ''
        self.position = Obj(x=0., y=0., z=0.)
        self.orientation = Obj(x=0., y=0., z=0., w=1.)


class Poses(list):
    def add(self):
        value = FakePose()
        self.append(value)
        return value


class FakePoseV:
    def __init__(self):
        self.pose = Poses()


class RuntimeChecks(unittest.TestCase):
    def run_animation(self, frozen=False, rejected=False, clock_rate=1.0, paused=False,
                      apply_delay=0.0, vector_outcomes=(), vector_default='ok',
                      single_outcomes=None, timeout_applies=False, coupled_timeouts=False):
        config = json.loads((PACKAGE/'step1/config.json').read_text())
        timer = Obj(wall=0.0, sim=0.0)
        subscriptions, requests, output = {}, [], io.StringIO()
        pose = Obj(name=config['model_name'], header=Obj(stamp=stamp(0.0)),
                   position=Obj(x=0., y=-14., z=.005),
                   orientation=Obj(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4)))
        pending = Obj(pose=None, due=0.0)
        vector_results = list(vector_outcomes)
        single_results = {name: list(results) for name, results in (single_outcomes or {}).items()}

        class Node:
            def __init__(self):
                self.subscriptions = self.calls = 0

            def subscribe(self, message_type, topic, callback):
                self.subscriptions += 1
                subscriptions[topic] = callback
                return True

            def request(self, service, message, request_type, response_type, timeout):
                if service == '/world/factory/set_pose_vector':
                    self_test.assertIs(request_type, FakePoseV)
                    self_test.assertEqual(len(message.pose), 3)
                    self_test.assertEqual({p.name for p in message.pose},
                                          {'aisle_robot', 'aisle_robot_left_wheel', 'aisle_robot_right_wheel'})
                    poses = message.pose
                    outcome = vector_results.pop(0) if vector_results else vector_default
                else:
                    self_test.assertEqual(service, '/world/factory/set_pose')
                    self_test.assertIs(request_type, FakePose)
                    self_test.assertIsInstance(message, FakePose)
                    poses = [message]
                    results = single_results.get(message.name, [])
                    outcome = results.pop(0) if results else 'ok'
                self_test.assertIn(timeout, (500, 2000, 5000))
                if rejected:
                    outcome = 'rejected'
                if coupled_timeouts and self.subscriptions and self.calls:
                    outcome = 'timeout'
                self.calls += 1
                requests.append(Obj(wall=timer.wall, service=service, poses=deepcopy(poses),
                                    timeout=timeout, outcome=outcome))
                body = next((p for p in poses if p.name == config['model_name']), None)
                if body is not None and not frozen and \
                        (outcome == 'ok' or (outcome == 'timeout' and timeout_applies)):
                    pending.pose = deepcopy(body)
                    pending.due = timer.wall+apply_delay
                if outcome == 'timeout':
                    sleep(timeout/1000)
                    return False, None
                if outcome == 'error':
                    raise RuntimeError('transport connection interrupted')
                return True, Obj(data=outcome != 'rejected')

        class RequestProcess:
            """Stand-in for the separate process; its Node has no subscriptions."""
            def __init__(self, log_path):
                self.node = Node()
                self.pid = 99999

            def request(self, *args):
                return self.node.request(*args)

            def close(self):
                pass

        self_test = self

        def sleep(seconds):
            nonlocal pose
            timer.wall += max(.001, seconds)
            timer.sim = timer.wall*clock_rate if not paused else 0.0
            if pending.pose is not None and timer.wall >= pending.due:
                pose, pending.pose = pending.pose, None
            pose.header = Obj(stamp=stamp(timer.sim))
            subscriptions['/world/factory/clock'](Obj(sim=stamp(timer.sim)))
            subscriptions['/world/factory/pose/info'](Obj(header=Obj(stamp=stamp(0.0)), pose=[pose]))
            if timer.wall > 125 or (paused and timer.wall > 1.5):
                raise KeyboardInterrupt

        with tempfile.TemporaryDirectory(prefix='step1_animation_check_') as directory:
            package = Path(directory)
            for relative in ('step1/config.json', 'step1/route.json', 'step1/map.json', 'worlds/warehouse.sdf'):
                destination = package/relative
                destination.parent.mkdir(exist_ok=True)
                shutil.copyfile(PACKAGE/relative, destination)
            with patch.object(animate_step1, 'PACKAGE', package), \
                    patch.object(animate_step1, 'transport_imports', return_value=(Node, FakePose, FakePoseV, object, object)), \
                    patch.object(animate_step1, 'PoseServiceProcess', RequestProcess, create=True), \
                    patch('sys.argv', ['animate_step1.py']), \
                    patch('time.monotonic', side_effect=lambda: timer.wall), \
                    patch('time.sleep', side_effect=sleep), redirect_stdout(output):
                error, result = None, None
                try:
                    result = animate_step1.main()
                except RuntimeError as exception:
                    error = str(exception)
            report = json.loads((package/'logs/step1_result.json').read_text())
        return report, result, error, requests, output.getvalue()

    def test_real_client_completes_the_entire_90_second_tour(self):
        report, result, error, requests, _ = self.run_animation()
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(set(report['visited_sections'])), 15)
        self.assertLess(report['finish_error_metres'], .025)
        self.assertGreaterEqual(report['elapsed_wall_seconds'], 90)
        self.assertLess(report['elapsed_wall_seconds'], 95)
        self.assertGreater(len(requests), 2000)
        for a, b in zip(requests, requests[1:]):
            self.assertLessEqual(math.hypot(a.poses[0].position.x-b.poses[0].position.x,
                                           a.poses[0].position.y-b.poses[0].position.y), .25+1e-8)

    def test_timeout_after_the_first_moving_update_recovers(self):
        report, result, error, requests, _ = self.run_animation(
            vector_outcomes=['ok', 'ok', 'timeout'])
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['elapsed_wall_seconds'], 120)
        self.assertTrue(any(r.outcome == 'timeout' for r in requests))
        self.assertEqual(report['pose_request_failures'], 1)
        self.assertEqual(report['pose_request_retries'], 1)
        self.assertEqual(report['pose_service'], '/world/factory/set_pose_vector')

    def test_requests_keep_working_when_a_subscribed_node_would_time_out(self):
        report, result, error, _, _ = self.run_animation(coupled_timeouts=True)
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['elapsed_wall_seconds'], 120)

    def test_batch_rejections_switch_to_single_pose_updates_for_all_three_models(self):
        report, result, error, requests, output = self.run_animation(
            vector_outcomes=['ok', 'ok'], vector_default='rejected')
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['elapsed_wall_seconds'], 120)
        self.assertEqual(report['pose_service'], '/world/factory/set_pose')
        self.assertIn('using /world/factory/set_pose', output)
        vector = [r for r in requests if r.service.endswith('set_pose_vector')]
        self.assertEqual(len(vector), 4)  # No repeated batch probing after switching.
        singles = [r for r in requests if r.service.endswith('/set_pose')]
        self.assertGreater(len(singles), 6000)
        self.assertEqual(len(singles) % 3, 0)
        for index in range(0, len(singles), 3):
            self.assertEqual([r.poses[0].name for r in singles[index:index+3]],
                             ['aisle_robot', 'aisle_robot_left_wheel', 'aisle_robot_right_wheel'])
        bodies = [r.poses[0] for r in singles if r.poses[0].name == 'aisle_robot']
        for a, b in zip(bodies, bodies[1:]):
            self.assertLessEqual(math.hypot(a.position.x-b.position.x, a.position.y-b.position.y), .25+1e-8)

    def test_missing_batch_service_uses_the_single_pose_service(self):
        report, result, error, requests, _ = self.run_animation(vector_default='timeout')
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertEqual(report['pose_service'], '/world/factory/set_pose')
        self.assertLess(requests[-1].wall, 120)

    def test_timed_out_but_applied_command_is_retried_without_skipping(self):
        report, result, error, requests, _ = self.run_animation(
            vector_outcomes=['ok', 'ok', 'timeout'], timeout_applies=True)
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['finish_error_metres'], .025)
        failed = next(i for i, r in enumerate(requests) if r.outcome == 'timeout')
        for a, b in zip(requests[failed].poses, requests[failed+1].poses):
            self.assertEqual(vars(a.position), vars(b.position))
            self.assertEqual(vars(a.orientation), vars(b.orientation))

    def test_single_wheel_timeout_recovers(self):
        report, result, error, requests, _ = self.run_animation(
            vector_default='rejected',
            single_outcomes={'aisle_robot_left_wheel': ['ok', 'ok', 'timeout']})
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['elapsed_wall_seconds'], 120)
        self.assertTrue(any(r.outcome == 'timeout' and
                            r.poses[0].name == 'aisle_robot_left_wheel' for r in requests))

    def test_rejected_wheel_stops_with_model_and_service_details(self):
        report, _, error, _, _ = self.run_animation(
            vector_default='rejected',
            single_outcomes={'aisle_robot_right_wheel': ['rejected', 'rejected']})
        self.assertFalse(report['completed'])
        self.assertEqual(report['visited_sections'], [])
        self.assertIn('aisle_robot_right_wheel', error)
        self.assertIn('/world/factory/set_pose', error)
        self.assertIn('data=false', error)

    def test_transport_error_after_the_first_move_recovers(self):
        report, result, error, _, _ = self.run_animation(vector_outcomes=['ok', 'ok', 'error'])
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)

    def test_slow_clock_stamps_do_not_slow_the_animation_budget(self):
        report, result, error, _, _ = self.run_animation(clock_rate=.1)
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertLess(report['elapsed_wall_seconds'], 95)
        self.assertLess(report['simulation_seconds'], 10)

    def test_delayed_pose_application_keeps_all_sections_and_stays_under_two_minutes(self):
        report, result, error, _, _ = self.run_animation(apply_delay=.04)
        self.assertIsNone(error)
        self.assertEqual(result, 0)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertLess(report['elapsed_wall_seconds'], 120)

    def test_accepted_but_unapplied_commands_do_not_fake_progress(self):
        report, _, error, _, _ = self.run_animation(frozen=True)
        self.assertFalse(report['completed'])
        self.assertEqual(report['visited_sections'], [])
        self.assertIn('did not reach it', error)

    def test_rejected_service_stops_the_animation(self):
        report, _, error, _, _ = self.run_animation(rejected=True)
        self.assertFalse(report['completed'])
        self.assertIn('Gazebo rejected the update', error)
        self.assertIn('/world/factory/set_pose', error)

    def test_paused_world_holds_outside_without_motion_commands(self):
        report, result, error, requests, output = self.run_animation(paused=True)
        self.assertIsNone(error)
        self.assertEqual(result, 130)
        self.assertFalse(report['completed'])
        self.assertIn('animation held', output)
        # Initial hold plus the brief interval before paused-clock detection.
        self.assertTrue(all(abs(r.poses[0].position.y+14) < .025 for r in requests))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--structural-only', action='store_true')
    args = parser.parse_args()
    config, route, grid = structural_check()
    if args.structural_only:
        return
    metrics = timeline_check(config, route, grid)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RuntimeChecks)
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    (PACKAGE/'step1/preview_validation.json').write_text(json.dumps({
        'passed': True, 'motion_mode': 'kinematic_route_animation',
        'gazebo_runtime_tested_here': False, 'runtime_regression_tests': result.testsRun,
        **metrics,
        'note': 'Fake transport checks the real client; Gazebo rendering and service integration need a native run.'
    }, indent=2)+'\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Verify beacon proximity, safe approaches, pauses, relay links and confirmed drops.

Offline checks exercise the real mission loop. They do not claim native Gazebo
rendering or transport validation on this machine.
"""
import argparse
from collections import Counter
from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import io
import json
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

from animate_step2 import Step2Connection, run_mission
from build_step2 import beacon_sdf, beacon_specs
from check_step1_preview import FakePoseV
from step1_feedback import Feedback
from step1_map import GridMap, read_geometry
from step2_events import MissionEvents, PACKAGE, event_gap, load_config, read_events
from step2_timeline import MissionTimeline


def normalized(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(normalized(child) for child in element))


def structural_check(config=None):
    config = config or load_config()
    route = json.loads((PACKAGE/'step2/route.json').read_text())
    hashes = {'world_sha256': 'worlds/warehouse.sdf', 'step1_route_sha256': 'step1/route.json',
              'config_sha256': 'step2/config.json', 'step1_config_sha256': 'step1/config.json'}
    for key, file in hashes.items():
        if route[key] != hashlib.sha256((PACKAGE/file).read_bytes()).hexdigest():
            raise RuntimeError('Mission inputs changed. Run bash run.sh plan first.')
    assert route['events'] == read_events(config)
    original = json.loads((PACKAGE/'step1/route.json').read_text())
    assert original['aisle_segments'] == route['aisle_segments']
    assert Counter(w['name'] for w in route['waypoints'] if w['kind'] == 'aisle') == \
        Counter(s['name'] for s in original['aisle_segments'])
    grid = GridMap.load(PACKAGE/'step2/map.json')
    for a, b in zip(route['waypoints'], route['waypoints'][1:]):
        assert grid.line_safe((a['x'], a['y']), (b['x'], b['y'])), 'Unsafe approach or connecting line'
    assert math.dist((route['waypoints'][0]['x'], route['waypoints'][0]['y']), config['start'][:2]) < 1e-8
    assert route['waypoints'][0] == original['waypoints'][0]
    assert route['waypoints'][-1] == original['waypoints'][-1]
    for w in route['waypoints']:
        if w['kind'] == 'event_approach':
            event = next(e for e in route['events'] if e['id'] == w['event_id'])
            gap = event_gap({'x': w['x'], 'y': w['y'], 'yaw': w['face_yaw']}, event)
            assert .1 < gap < .30, ('Victim approach cannot satisfy 30 cm', gap)
    source = ET.parse(PACKAGE/'worlds/warehouse_step1_preview.sdf').getroot().find('world')
    target = ET.parse(PACKAGE/'worlds/warehouse_step2.sdf').getroot().find('world')
    assert [normalized(m) for m in source.findall('model')] == \
        [normalized(m) for m in target.findall('model') if not m.get('name').startswith('step2_')], \
        'Step 2 changed the working environment or robot'
    models = {m.get('name'): m for m in target.findall('model')}
    assert len(models) == len(target.findall('model')), 'Duplicate model names'
    assert route['beacon_deployment'] == 'fixed_entrance_and_create_event_tiles'
    assert set(route['pose_models']) & set(models) == {'step2_entrance_blue_beacon'}
    assert {name for name in models if name.startswith('step2_')} == \
        {'step2_outside_gateway', 'step2_entrance_blue_beacon'}, \
        'Startup must contain the entrance assembly and outside box only'
    for name, kind in beacon_specs(route['events'], config):
        m = ET.fromstring(beacon_sdf(name, kind, config['entrance_beacon_position'], config)).find('model')
        assert m.findtext('static') == ('true' if kind == 'gateway' else 'false')
        assert m.findtext('link/gravity') == 'false'
        assert not m.findall('link/collision')
        assert float(m.findtext('pose').split()[2])-config['beacon_size_metres'][2]/2 > 0, \
            'A created beacon extends below the floor'
        assert list(map(float, m.findtext('link/visual/geometry/box/size').split())) == \
            config['beacon_size_metres']
        assert m.find('plugin') is not None
    cable = models['step2_entrance_blue_beacon']
    expected = ET.fromstring(beacon_sdf('step2_entrance_blue_beacon', 'gateway',
                                      config['entrance_beacon_position'], config)).find('model')
    assert normalized(cable) == normalized(expected), 'Startup entrance assembly is missing or misplaced'
    gx, gy, gz = config['entrance_beacon_position']
    assert gy == -12.0+1.0 and config['entrance_robot_stop'][1] == gy, \
        'Gateway is not one metre inside the doorway'
    assert cable.findtext('plugin/topic') == '/model/step2_entrance_blue_beacon/pose'
    assert cable.findtext('plugin/static_publisher') == 'false'
    assert {v.get('name') for v in cable.findall('link/visual')} == {'tile', 'cable_0', 'cable_1'}
    endpoints = []
    for visual in cable.findall('link/visual'):
        assert visual.findtext('transparency') == '0'
        assert visual.findtext('visibility_flags') == '4294967295'
        if visual.get('name') == 'tile':
            continue
        x, y, z, _, _, yaw = map(float, visual.findtext('pose').split())
        length, _, thickness = map(float, visual.findtext('geometry/box/size').split())
        assert gz+z-thickness/2 >= .0075-1e-8, 'Entrance wire obscured by the floor'
        endpoints.append([(gx+x+s*length/2*math.cos(yaw), gy+y+s*length/2*math.sin(yaw))
                          for s in (-1, 1)])
    nx, ny, _ = config['outside_gateway_position']
    assert math.dist(endpoints[0][0], (nx, ny+.175)) < 1e-8
    assert math.dist(endpoints[0][1], endpoints[1][0]) < 1e-8
    assert math.dist(endpoints[-1][1], config['entrance_beacon_position'][:2]) < 1e-8
    box = models['step2_outside_gateway']
    assert float(box.findtext('pose').split()[1]) < -12.0
    assert box.findtext('static') == 'true'
    assert len(route['aisle_segments']) == 15
    assert Counter(e['kind'] for e in route['events']) == {'fire': 6, 'victim': 2}
    dimensions = ' x '.join(f'{v*100:g}' for v in config['beacon_size_metres'])
    print(f'PASS: working scene preserved; safe victim approaches; {dimensions} cm tiles; '
          'entrance tile and cable present at startup, 1 m inside; all 15 aisles.')
    return route, grid


class Timer:
    def __init__(self):
        self.wall = 0.0
    def monotonic(self):
        return self.wall
    def sleep(self, seconds):
        self.wall += max(.00001, seconds)


class FakeFactory:
    def __init__(self):
        self.name, self.sdf, self.allow_renaming = '', '', False


class CreationFixture:
    """Render-independent world creation, with actual model-pose callbacks."""
    def __init__(self, connection):
        self.connection, self.calls, self.models = connection, [], {}
        entrance = ET.parse(PACKAGE/'worlds/warehouse_step2.sdf').find(
            './/model[@name="step2_entrance_blue_beacon"]')
        root = ET.Element('sdf', version='1.9')
        root.append(entrance)
        initial = FakeFactory()
        initial.name, initial.sdf = entrance.get('name'), ET.tostring(root, encoding='unicode')
        self.models[initial.name] = initial

    def feedback(self, message):
        model = ET.fromstring(message.sdf).find('model')
        xyz = list(map(float, model.findtext('pose').split()))[:3]
        vector = FakePoseV()
        Step2Connection.fill_pose(vector.pose.add(), ('factory::'+message.name, *xyz, 0, 0, 0, 1))
        source = f'/model/{message.name}/pose' if model.findtext('static') == 'true' \
            else '/world/factory/dynamic_pose/info'
        self.connection.receive(vector, source)

    def request(self, service, message, request_type, response_type, timeout):
        assert service == '/world/factory/create'
        assert request_type is FakeFactory
        assert not message.allow_renaming
        self.calls.append(deepcopy(message))
        if message.name in self.models:
            self.feedback(self.models[message.name])
            return True, SimpleNamespace(data=False)
        model = ET.fromstring(message.sdf).find('model')
        assert model.get('name') == message.name
        self.models[message.name] = deepcopy(message)
        self.feedback(message)
        return True, SimpleNamespace(data=True)


def fixture_connection(connection=None, config=None):
    connection = connection or Step2Connection.__new__(Step2Connection)
    connection.config = config or load_config()
    Feedback.__init__(connection, {**connection.config, 'prefer_newest_pose': True})
    connection.EntityFactory, connection.Boolean = FakeFactory, object
    connection.model_names = {name for name, _ in beacon_specs(read_events(connection.config), connection.config)}
    connection.model_samples, connection.created_models = {}, set()
    connection.tracked_models = connection.model_names | {connection.name, connection.name+'_left_wheel',
                                                        connection.name+'_right_wheel'}
    connection.entity_ids, connection.feedback_process = {}, None
    connection.motion_recoveries, connection.last_motion_mismatch = 0, None
    connection.command_pose_time = None
    connection.use_vector, connection.fallback_reason = True, None
    connection.create_service = '/world/factory/create'
    connection.create_failures = connection.create_retries = 0
    connection.last_create_failure = None
    connection.requester = CreationFixture(connection)
    return connection


class MissionChecks(unittest.TestCase):
    def mission(self, apply_delay=0.0, paused=False, fail_drop=False, missing_events=False, radio_range=None,
                lost_resume=False, never_resume=False):
        config = load_config()
        if radio_range is not None:
            config['communication_range_metres'] = radio_range
        route, grid = structural_check(config)
        timer = Timer()
        floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
        mission = MissionEvents(config, route['events'], floors, obstacles, fires)
        report, output = {'completed': False}, io.StringIO()
        timeline = MissionTimeline(route['waypoints'], config, config['motion_seconds'])
        initial = timeline.sample(0.0)
        self_test = self

        class Connection(Step2Connection):
            def __init__(self):
                fixture_connection(self, config)
                self.pose, self.pending, self.due = initial, None, 0.0
                self.sent, self.placements = [], []
                self.paused_seconds = 0.0
            def state(self):
                if self.pending is not None and timer.wall >= self.due:
                    self.pose, self.pending = self.pending, None
                sim = timer.wall
                if paused and 5 <= timer.wall < 7:
                    sim = 5.0
                elif paused and timer.wall >= 7:
                    sim -= 2.0
                return {**self.pose, 'z': .005, 'sim_time': sim,
                        'pose_time': sim,
                        'pose_fresh': True, 'time_reset': False}
            def send(self, pose):
                self.command_pose_time = self.state()['sim_time']+2*config['preview_physics_step']
                if self.sent:
                    previous = self.sent[-1][1]
                    self_test.assertLessEqual(math.dist((previous['x'], previous['y']),
                                                       (pose['x'], pose['y'])), .250001)
                    self_test.assertLessEqual(abs(previous['yaw']-pose['yaw']), .200001)
                self.sent.append((timer.wall, deepcopy(pose)))
                # Reproduce the user's accepted first command after deploying
                # the gateway and wire without any corresponding movement.
                if 'step2_entrance_blue_beacon' in self.created_models and \
                        (never_resume or (lost_resume and not self.motion_recoveries)):
                    return
                self.pending, self.due = deepcopy(pose), timer.wall+apply_delay
            def place(self, event):
                if fail_drop:
                    raise RuntimeError('Beacon feedback missing')
                self.placements.append((timer.wall, deepcopy(event), deepcopy(self.pose)))
                self_test.assertTrue(mission.tile_safe(event['position'][:2], self.pose,
                                                      event['kind'] != 'gateway'))
                super().place(event)
                if missing_events:
                    mission.done.update(e['id'] for e in route['events'])
        connection = Connection()
        with redirect_stdout(output):
            run_mission(connection, config, route, grid, mission, config['motion_seconds'],
                        report, clock=timer)
        return config, route, mission, connection, report, output.getvalue()

    def test_complete_mission_uses_confirmed_pose_and_keeps_all_beacons(self):
        config, route, mission, connection, report, _ = self.mission()
        self.assertTrue(report['completed'])
        counts = Counter(b['kind'] for b in mission.beacons)
        self.assertEqual(counts['gateway'], 1)
        self.assertEqual(counts['fire'], 6)
        self.assertEqual(counts['victim'], 2)
        self.assertGreater(counts['range'], 0)
        self.assertLess(report['elapsed_wall_seconds'], 120.0)
        self.assertLess(report['finish_error_metres'], .00001)
        self.assertEqual(len(mission.records), len({b['name'] for b in mission.beacons}))
        inbox = report['communication']['received_messages']
        self.assertEqual({packet['message_id'] for packet in inbox},
                         {record['id'] for record in mission.records})
        self.assertEqual(len(inbox), len(mission.records))
        for packet in inbox:
            self.assertEqual(packet['status'], 'received')
            self.assertEqual(packet['route'][-2:],
                             ['step2_entrance_blue_beacon', 'step2_outside_gateway'])
            self.assertEqual(packet['links'][-1]['medium'], 'cable')
            self.assertTrue(all(link['medium'] == 'wireless' and
                                link['distance_metres'] <= config['communication_range_metres']
                                for link in packet['links'][:-1]))
        victim_packets = [p for p in inbox if p['payload']['kind'] == 'victim']
        self.assertEqual({p['payload']['victim_status'] for p in victim_packets}, {'alive', 'dead'})
        self.assertEqual(set(connection.requester.models), {b['name'] for b in mission.beacons})
        self.assertNotIn('step2_entrance_blue_beacon', [m.name for m in connection.requester.calls])
        for name, definition in connection.requester.models.items():
            model = ET.fromstring(definition.sdf).find('model')
            size = list(map(float, model.findtext('link/visual/geometry/box/size').split()))
            self.assertEqual(size, [.10, .10, .03])
            self.assertGreater(float(model.findtext('pose').split()[2])-size[2]/2, 0.0)
        events = {e['id']: e for e in route['events']}
        for record in mission.records:
            self.assertGreaterEqual(record['pause_seconds']+1e-8, 1.0)
            if record['kind'] in ('fire', 'victim'):
                x, y, yaw = record['robot_pose']
                self.assertLessEqual(event_gap({'x': x, 'y': y, 'yaw': yaw}, events[record['id']]),
                                     events[record['id']]['distance']+1e-8)
            if record['kind'] != 'gateway':
                self.assertLessEqual(record['parent_distance_metres'], config['communication_range_metres'])
            else:
                self.assertEqual(record['position'], config['entrance_beacon_position'])
        for wall, _, _ in connection.placements:
            moving = [t for t, state in connection.sent if wall-1.0+1e-8 < t < wall]
            self.assertFalse(moving, 'Robot commanded motion during the one-second drop pause')
        (PACKAGE/'step2/offline_mission.json').write_text(json.dumps({**report,
            'native_gazebo_runtime': False,
            'beacon_deployment': 'fixed_entrance_and_create_event_tiles',
            'note': 'Real mission loop and beacon creation client with deterministic transport/pose fixtures.'}, indent=2)+'\n')

    def test_delayed_feedback_does_not_skip_approaches_or_duplicate_events(self):
        _, _, mission, connection, report, _ = self.mission(apply_delay=.06)
        self.assertTrue(report['completed'])
        self.assertEqual(len(report['visited_sections']), 15)
        self.assertEqual(sum(r['kind'] == 'victim' for r in mission.records), 2)
        self.assertLess(report['elapsed_wall_seconds'], 180,
                        'Feedback waits erased the motion budget and made the tour crawl')

    def test_accepted_but_unapplied_resume_recovers_and_finishes_all_events(self):
        _, _, mission, connection, report, output = self.mission(lost_resume=True)
        self.assertTrue(report['completed'])
        self.assertEqual(connection.motion_recoveries, 1)
        self.assertIn('Movement recovery', output)
        counts = Counter(b['kind'] for b in mission.beacons)
        self.assertEqual({k: counts[k] for k in ('gateway', 'fire', 'victim')},
                         {'gateway': 1, 'fire': 6, 'victim': 2})
        self.assertGreater(counts['range'], 0)
        self.assertEqual(len(connection.requester.models), len(mission.beacons))
        self.assertEqual(len(report['visited_sections']), 15)

    def test_permanently_unapplied_movement_stops_with_actual_target_diagnostic(self):
        with self.assertRaisesRegex(RuntimeError, 'Observed .*target .*step2_result.json'):
            self.mission(never_resume=True)

    def test_paused_clock_holds_motion_and_preserves_beacons(self):
        _, _, mission, connection, report, _ = self.mission(paused=True)
        self.assertTrue(report['completed'])
        self.assertFalse([t for t, _ in connection.sent if 5.6 < t < 6.9])

    def test_unconfirmed_beacon_cannot_be_committed(self):
        with self.assertRaisesRegex(RuntimeError, 'Beacon feedback missing'):
            self.mission(fail_drop=True)

    def test_scene_box_is_not_a_radio_source(self):
        config = load_config()
        floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
        mission = MissionEvents(config, read_events(config), floors, obstacles, fires)
        self.assertIsNone(mission.nearest({'x': -1.25, 'y': -13.15})[0])

    def test_complete_requires_each_fire_and_victim(self):
        config = load_config()
        floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
        mission = MissionEvents(config, read_events(config), floors, obstacles, fires)
        mission.done.add('gateway')
        mission.range_count = 1
        with self.assertRaisesRegex(RuntimeError, 'Missing beacon events'):
            mission.require_complete()

    def test_large_radio_range_only_drops_relays_if_needed(self):
        _, _, mission, _, report, _ = self.mission(radio_range=30.0)
        self.assertTrue(report['completed'])
        self.assertEqual(mission.range_count, 0)
        self.assertEqual(len(mission.beacons), 9)


class DropClientChecks(unittest.TestCase):
    def connection(self):
        return fixture_connection()

    def event(self):
        event = read_events(load_config())[0]
        return {'id': event['id'], 'kind': 'fire', 'beacon_name': 'step2_'+event['id']+'_beacon',
                'position': load_config()['entrance_beacon_position']}

    def entrance_event(self):
        return {'id': 'gateway', 'kind': 'gateway', 'beacon_name': 'step2_entrance_blue_beacon',
                'position': load_config()['entrance_beacon_position']}

    def test_entrance_is_preloaded_and_needs_no_creation_or_pose_subscription(self):
        connection = self.connection()
        connection.place(self.entrance_event())
        self.assertEqual(connection.requester.calls, [])
        tile = ET.fromstring(connection.requester.models['step2_entrance_blue_beacon'].sdf).find('model')
        self.assertEqual(list(map(float, tile.findtext('pose').split()))[:3],
                         self.entrance_event()['position'])
        self.assertEqual(tile.findtext('static'), 'true')
        self.assertEqual({v.get('name') for v in tile.findall('link/visual')},
                         {'tile', 'cable_0', 'cable_1'})
        self.assertEqual(connection.created_models, {'step2_entrance_blue_beacon'})

    def test_retry_keeps_same_name_and_disables_automatic_renaming(self):
        connection = self.connection()
        original, calls, timer = connection.requester.request, [], Timer()
        def request(*args):
            calls.append(deepcopy(args[1]))
            if len(calls) == 1:
                return False, None
            return original(*args)
        connection.requester.request = request
        with patch('animate_step2.time', timer), redirect_stdout(io.StringIO()):
            connection.place(self.event())
        self.assertEqual([m.name for m in calls],
                         [self.event()['beacon_name']]*2)
        self.assertTrue(all(not m.allow_renaming for m in calls))
        self.assertEqual(connection.create_retries, 1)
        self.assertEqual(len(connection.requester.models), 2)

    def test_lost_ack_after_successful_creation_does_not_duplicate_the_model(self):
        connection, timer = self.connection(), Timer()
        original = connection.requester.request
        def request(*args):
            original(*args)
            return False, None
        connection.requester.request = request
        with patch('animate_step2.time', timer):
            connection.place(self.event())
        self.assertEqual(len(connection.requester.calls), 1)
        self.assertEqual(len(connection.requester.models), 2)

    def test_repeated_entrance_activation_never_recreates_the_assembly(self):
        connection = self.connection()
        connection.place(self.entrance_event())
        connection.place(self.entrance_event())
        self.assertEqual(len(connection.requester.calls), 0)
        self.assertEqual(len(connection.requester.models), 1)

    def test_creation_after_timeout_with_late_feedback_remains_unique(self):
        connection, timer = self.connection(), Timer()
        original, calls = connection.requester.request, []
        def request(*args):
            message = args[1]
            calls.append(deepcopy(message))
            if len(calls) == 1:
                connection.requester.models[message.name] = deepcopy(message)
                return False, None
            return original(*args)
        connection.requester.request = request
        with patch('animate_step2.time', timer), redirect_stdout(io.StringIO()):
            connection.place(self.event())
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(connection.requester.models), 2)
        self.assertEqual(connection.created_models, {self.event()['beacon_name']})

    def test_positive_service_acknowledgement_without_beacon_feedback_fails(self):
        connection, timer = self.connection(), Timer()
        connection.requester.request = lambda *args: (True, SimpleNamespace(data=True))
        with patch('animate_step2.time', timer):
            with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
                connection.place(self.event())

    def test_delayed_event_tile_confirmation_is_required(self):
        connection, timer = self.connection(), Timer()
        pending, original = {}, connection.applied
        def request(service, message, *args):
            pending[message.name] = (timer.wall+.3, deepcopy(message))
            return True, SimpleNamespace(data=True)
        def applied(values):
            for name, (due, message) in list(pending.items()):
                if timer.wall >= due:
                    connection.requester.feedback(message)
                    del pending[name]
            return original(values)
        connection.requester.request, connection.applied = request, applied
        with patch('animate_step2.time', timer):
            connection.place(self.event())
        self.assertGreaterEqual(timer.wall, .3)
        self.assertEqual(connection.created_models, {self.event()['beacon_name']})

    def test_feedback_below_the_floor_cannot_confirm_an_event_drop(self):
        connection, timer = self.connection(), Timer()
        def request(service, message, *args):
            underground = deepcopy(message)
            root = ET.fromstring(underground.sdf)
            root.find('model/pose').text = '0 0 -20 0 0 0'
            underground.sdf = ET.tostring(root, encoding='unicode')
            connection.requester.feedback(underground)
            return True, SimpleNamespace(data=True)
        connection.requester.request = request
        with patch('animate_step2.time', timer):
            with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
                connection.place(self.event())

    def test_underground_spawn_definition_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'above the floor'):
            beacon_sdf('step2_entrance_blue_beacon', 'gateway', [-.45, -11, -20], load_config())

    def test_tile_half_buried_at_zero_height_cannot_be_confirmed(self):
        connection, timer = self.connection(), Timer()
        with patch('animate_step2.time', timer):
            connection.model_samples['step2_entrance_blue_beacon'] = (-.45, -11, 0, 0)
            self.assertFalse(connection.applied([('step2_entrance_blue_beacon',
                                                  *self.entrance_event()['position'])]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--structural-only', action='store_true')
    args = parser.parse_args()
    route, grid = structural_check()
    if args.structural_only:
        return
    suite = unittest.TestSuite()
    for cls in (MissionChecks, DropClientChecks):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(cls))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    (PACKAGE/'step2/validation.json').write_text(json.dumps({
        'passed': True, 'tests': result.testsRun, 'native_gazebo_runtime_tested_here': False,
        'motion_feedback_and_beacon_services': 'offline fixtures',
        'beacon_deployment': 'fixed_entrance_and_create_event_tiles',
        'entrance_tile_and_cable_present_at_startup': True,
        'entrance_activation_requires_no_runtime_creation': True,
            'initial_world_has_no_underground_staging': True,
            'all_beacon_information_received_by_outside_gateway_through_cable': True,
        'event_distance_convention': 'horizontal robot-edge to victim-edge / excluded fire region'
    }, indent=2)+'\n')


if __name__ == '__main__':
    main()

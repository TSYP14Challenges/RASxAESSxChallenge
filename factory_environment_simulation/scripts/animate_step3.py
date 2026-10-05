#!/usr/bin/env python3
"""Run the working writer first, then the gray robot's eight beacon actions."""
import argparse
import csv
import json
import math
import sys
import time

from animate_step2 import Step2Connection, creation_message_type, pose_matches, run_mission
from build_step3 import ball_name, ball_sdf
from step1_map import read_geometry
from step2_events import PACKAGE, load_config as writer_config
from step2_timeline import MissionTimeline
from step3_scenario import AccessibleWriterEvents, ball_point, load_config, plan_executor


def extra_message_types():
    # animate_step2 selects the Python parser and imports Transport first.
    creation_message_type()
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.visual_pb2 import Visual
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.empty_pb2 import Empty
    entity = Entity(name='fire_01', type=Entity.MODEL)
    decoded = Entity()
    decoded.ParseFromString(entity.SerializeToString())
    if decoded.name != 'fire_01' or decoded.type != Entity.MODEL:
        raise ValueError('Gazebo removal message does not decode')
    visual = Visual(id=123, parent_id=122, name='tile', visible=True)
    visual.material.diffuse.r, visual.material.diffuse.g = .05, .85
    visual.material.diffuse.b, visual.material.diffuse.a = .15, 1.
    copied = Visual()
    copied.ParseFromString(visual.SerializeToString())
    if copied.id != 123 or abs(copied.material.diffuse.g-.85) > 1e-6:
        raise ValueError('Gazebo beacon color message does not decode')
    scene = Scene()
    model = scene.model.add(name='step2_victim_alive_beacon', id=120)
    link = model.link.add(name='body', id=122)
    link.visual.add().CopyFrom(visual)
    copied_scene = Scene()
    copied_scene.ParseFromString(scene.SerializeToString())
    if copied_scene.model[0].link[0].visual[0].id != 123:
        raise ValueError('Gazebo scene identities do not decode')
    return Entity, Visual, Scene, Empty


class Step3Connection(Step2Connection):
    def __init__(self, config, model_names):
        self.Entity, self.Visual, _, _ = extra_message_types()
        super().__init__(config, model_names, worker_prefix='step3')
        self.scene_service = f'/world/{config["world_name"]}/scene/info'
        self.visual_service = f'/world/{config["world_name"]}/visual_config'
        self.remove_service = f'/world/{config["world_name"]}/remove'
        self.victim_visuals = {}

    def scene_models(self, names):
        return self.requester.scene_models(self.scene_service, names)

    def confirm_writer_parked(self, clock=time):
        name, start = writer_config()['model_name'], writer_config()['start']
        deadline = clock.monotonic()+10.
        while True:
            self.state()
            with self.lock:
                sample = self.model_samples.get(name)
            if sample and math.dist(sample[:2], start[:2]) <= .025 \
                    and abs(sample[2]-.005) <= .002 and clock.monotonic()-sample[3] <= 1.:
                return
            if clock.monotonic() > deadline:
                raise RuntimeError('Writer has not been confirmed outside; gray robot remains parked')
            clock.sleep(.02)

    def set_ball_pose(self, name, position, clock=time):
        message = self.Pose()
        self.prepare_pose(message, (name, *position, 0., 0., 0., 1.))
        if not self.request_pose(self.single_service, message, self.Pose, name):
            raise RuntimeError('Could not animate the fire ball '+name)
        deadline = clock.monotonic()+10.
        while not self.applied([(name, *position)]):
            self.state()
            if clock.monotonic() >= deadline:
                raise RuntimeError('Fire ball pose was not confirmed: '+name)
            clock.sleep(.005)

    def throw_ball(self, action, pose, clock=time):
        config = self.config
        name = ball_name(action['id'])
        start = [pose['x']+.25*math.cos(pose['yaw']), pose['y']+.25*math.sin(pose['yaw']), .70]
        finish = [*action['event_centre'], config['ball_radius_metres']+.01]
        self.create_and_confirm(name, ball_sdf(name, start, config), start)
        frames = max(2, math.ceil(config['ball_flight_seconds']*config['preview_frame_rate']))
        period = config['ball_flight_seconds']/frames
        for frame in range(1, frames+1):
            began = clock.monotonic()
            position = ball_point(start, finish, frame/frames, config['ball_arc_height_metres'])
            self.set_ball_pose(name, position, clock)
            clock.sleep(max(0., period-(clock.monotonic()-began)))
        return name

    def remove_fire(self, name, clock=time):
        current = self.scene_models([name]).get(name)
        if current is None:
            raise RuntimeError('The target fire is absent before extinguishing: '+name)
        message = self.Entity(name=name, id=current['id'], type=self.Entity.MODEL)
        if not self.request_pose(self.remove_service, message, self.Entity, name):
            raise RuntimeError('Could not remove the burning fire effect: '+name)
        deadline = clock.monotonic()+10.
        while name in self.scene_models([name]):
            if clock.monotonic() >= deadline:
                raise RuntimeError('Burning fire effect did not disappear: '+name)
            clock.sleep(.02)
        definition = (PACKAGE/'step3'/('extinguished_'+name+'.sdf')).read_text()
        import xml.etree.ElementTree as ET
        xyz = list(map(float, ET.fromstring(definition).findtext('model/pose').split()))[:3]
        with self.lock:
            self.model_samples.pop(name, None)
            self.entity_ids.pop(name, None)
        self.create_and_confirm(name, definition, xyz)

    def color_victim(self, action):
        name = action['beacon_name']
        model = self.scene_models([name]).get(name)
        tile = None if model is None else model['visuals'].get('body::tile')
        if not tile or not tile['id']:
            raise RuntimeError('Cannot find the existing white beacon visual: '+name)
        status = action['victim_status']
        color = self.config['alive_beacon_color' if status == 'alive' else 'dead_beacon_color']
        message = self.Visual(id=tile['id'], parent_id=tile['parent_id'], name='tile',
                              parent_name=name+'::body', visible=True, transparency=0.)
        for attribute, values in (('ambient', color), ('diffuse', color),
                                  ('emissive', [v*.12 for v in color[:3]]+[1.])):
            target = getattr(message.material, attribute)
            target.r, target.g, target.b, target.a = values
        if not self.request_pose(self.visual_service, message, self.Visual, name):
            raise RuntimeError('Gazebo rejected the victim beacon color change: '+name)
        self.victim_visuals[name] = tile['id']
        return {'beacon_color': 'green' if status == 'alive' else 'red',
                'visual_id': tile['id'], 'beacon_changed_in_place': True,
                'appearance_command_accepted': True}

    def perform_action(self, action, pose, clock=time):
        if math.dist((pose['x'], pose['y']), action['position'][:2]) > .002:
            raise RuntimeError('Gray robot must stand above the beacon before acting')
        if action['kind'] == 'fire':
            name = self.throw_ball(action, pose, clock)
            self.remove_fire(action['id'], clock)
            return {'result': 'fire_extinguished', 'ball_name': name, 'fire_on': False,
                    'charred_objects_preserved': True, 'beacon_color': 'orange'}
        if action['kind'] == 'victim':
            return {'result': 'victim_marked', 'victim_status': action['victim_status'],
                    **self.color_victim(action)}
        raise RuntimeError('Blue and yellow beacons are not action targets')

    def verify_beacons(self, records):
        names = [b['beacon_name'] for b in records]
        scene = self.scene_models(names)
        if set(scene) != set(names):
            raise RuntimeError('A writer beacon was removed during the gray robot mission')
        for name, original in self.victim_visuals.items():
            if scene[name]['visuals']['body::tile']['id'] != original:
                raise RuntimeError('Victim tile was recreated instead of recolored in place')


def run_executor(connection, config, plan, grid, report, clock=time, trace=None, checkpoint=None):
    timeline = MissionTimeline(plan['waypoints'], config, plan['motion_seconds'], config['maximum_speed'])
    print('STEP 3: gray robot; orange fire actions and white victim actions.', flush=True)
    print('Blue/yellow beacons are avoided and stay in place.', flush=True)
    began = clock.monotonic()
    while True:
        sample = connection.state()
        if sample and sample['pose_fresh']:
            break
        if clock.monotonic()-began > 90.:
            raise RuntimeError('Gray robot pose or Gazebo clock is missing')
        clock.sleep(.03)
    issued = timeline.sample(0.)
    if not pose_matches(sample, issued):
        raise RuntimeError('Gray robot is not at its outside start; restart the complete scenario')
    connection.confirm_writer_parked(clock)
    report.update(writer_handoff_confirmed=True, actions=[], ignored_beacons=plan['ignored_beacons'],
                  requested_motion_seconds=plan['motion_seconds'], route_metres=timeline.distance)
    connection.send(issued)
    first = last = clock.monotonic()
    previous_clock = sample['sim_time']
    clock_wall = issued_wall = recovery_wall = first
    progress = budget = 0.
    arrivals = frames = recoveries = 0
    pending, stalled = None, None
    pause_elapsed = travelled = 0.
    previous_xy = sample['x'], sample['y']
    actions = {action['id']: action for action in plan['actions']}
    visited = set()
    print(f'Gray route: {timeline.distance:.1f} m; {plan["motion_seconds"]:g} motion seconds; '
          f'{len(actions)} beacon stops.', flush=True)
    while True:
        frame_start = clock.monotonic()
        sample, now = connection.state(), clock.monotonic()
        delta, last = now-last, now
        if sample is None or sample['time_reset']:
            raise RuntimeError('Simulation reset during the gray robot mission')
        if sample['sim_time'] > previous_clock:
            clock_wall, previous_clock = now, sample['sim_time']
        if now-clock_wall > .5 or not sample['pose_fresh']:
            if stalled is None:
                stalled = now
                print('Waiting for Play or fresh gray robot feedback; motion held.', flush=True)
            if now-clock_wall <= .5 and now-stalled > 10.:
                raise RuntimeError('Clock advances but gray robot feedback is stale')
            clock.sleep(.02)
            continue
        stalled = None
        if pending is None:
            budget += delta
        if not connection.motion_confirmed(sample, issued):
            if now-issued_wall > 10.:
                raise RuntimeError(f'Gray robot did not reach target ({issued["x"]:.3f}, {issued["y"]:.3f}); '
                                   f'observed ({sample["x"]:.3f}, {sample["y"]:.3f})')
            if recoveries < 3 and now-recovery_wall >= (1. if not recoveries else 2.):
                connection.recover_motion(issued, sample)
                recoveries += 1
                recovery_wall = last = clock.monotonic()
            clock.sleep(.005)
            continue
        xy = sample['x'], sample['y']
        if not grid.line_safe(previous_xy, xy):
            raise RuntimeError('Gray robot left the safe path or crossed a blue/yellow beacon')
        travelled += math.dist(previous_xy, xy)
        previous_xy = xy
        just_arrived = False
        while arrivals < len(timeline.arrivals) and timeline.arrivals[arrivals][0] <= progress+1e-9:
            _, _, kind, name = timeline.arrivals[arrivals]
            arrivals += 1
            if kind == 'action':
                if pending is not None or name in visited:
                    raise RuntimeError('Duplicate or overlapping gray robot action')
                pending, pause_elapsed = actions[name], 0.
                just_arrived = True
                print('ABOVE BEACON: '+pending['beacon_name']+'; pausing 1 second.', flush=True)
        if pending is not None:
            if not just_arrived:
                pause_elapsed += delta
            if pause_elapsed+1e-9 < config['action_pause_seconds']:
                clock.sleep(1/config['preview_frame_rate'])
                continue
            outcome = connection.perform_action(pending, sample, clock)
            after = connection.state()
            if not after or not after['pose_fresh'] or not connection.motion_confirmed(after, issued):
                raise RuntimeError('Gray robot moved or its pose became stale during its beacon action')
            record = {'id': pending['id'], 'kind': pending['kind'], 'beacon_name': pending['beacon_name'],
                      'beacon_position': pending['position'], 'robot_pose': [sample['x'], sample['y'], sample['yaw']],
                      'pause_seconds': round(pause_elapsed, 3),
                      'elapsed_wall_seconds': round(clock.monotonic()-first, 3), **outcome}
            report['actions'].append(record)
            visited.add(pending['id'])
            if checkpoint:
                checkpoint(report)
            label = 'FIRE OFF' if pending['kind'] == 'fire' else 'VICTIM '+outcome['beacon_color'].upper()
            print(label+': '+pending['id']+'; '+pending['beacon_name']+' remains in place.', flush=True)
            if trace:
                trace.writerow([clock.monotonic()-first, sample['sim_time'], *xy, sample['yaw'],
                                progress, label, pending['id']])
            pending, last = None, clock.monotonic()
            clock.sleep(.001)
            continue
        if progress >= timeline.duration-1e-9:
            if visited != set(actions):
                raise RuntimeError('Gray robot missed an orange or white beacon')
            report.update(completed=True, elapsed_wall_seconds=round(now-first, 3),
                          travelled_metres=round(travelled, 3), frames_applied=frames,
                          finish_error_metres=math.dist(xy, config['start'][:2]))
            print('COMPLETE STEP 3: six fires off; left victim green; right victim red; gray robot outside.', flush=True)
            return report
        following = timeline.advance(progress, max(0., min(timeline.duration, budget)-progress))
        if following > progress:
            progress = following
            issued = timeline.sample(progress)
            connection.send(issued)
            issued_wall = recovery_wall = clock.monotonic()
            recoveries = 0
            frames += 1
            if trace:
                trace.writerow([now-first, sample['sim_time'], *xy, sample['yaw'], progress, issued['mode'], ''])
        clock.sleep(max(.001, 1/config['preview_frame_rate']-(clock.monotonic()-frame_start)))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n')


def run_writer(args, scenario):
    from check_step2 import structural_check
    config = writer_config()
    config.update(communication_range_metres=args.radio_range, motion_seconds=args.writer_seconds)
    route, grid = structural_check(config)
    floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    mission = AccessibleWriterEvents(config, route['events'], floors, obstacles, fires)
    report = {'completed': False, 'native_gazebo_runtime': True, 'beacons': [],
              'communication': mission.network.snapshot()}
    scenario['writer'] = report
    logs = PACKAGE/'logs'
    def checkpoint(data):
        write_json(logs/'step2_result.json', data)
        write_json(logs/'step2_events.json', data['beacons'])
        write_json(logs/'gateway_inbox.json', mission.network.snapshot())
        write_json(logs/'step3_result.json', scenario)
    connection = Step2Connection(config, route['pose_models'])
    try:
        checkpoint(report)
        with (logs/'step2_trace.csv').open('w', newline='') as stream:
            trace = csv.writer(stream)
            trace.writerow(['wall_elapsed', 'sim_time', 'x', 'y', 'yaw', 'motion_elapsed', 'mode', 'event'])
            run_mission(connection, config, route, grid, mission, args.writer_seconds, report,
                        trace=trace, checkpoint=checkpoint)
    except BaseException as error:
        report['reason'] = str(error)
        raise
    finally:
        report.update(connection.diagnostics())
        checkpoint(report)
        connection.close()
    return report, mission.records


def main():
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--writer-seconds', type=float, default=writer_config()['motion_seconds'])
    parser.add_argument('--seconds', type=float, default=config['motion_seconds'])
    parser.add_argument('--range', dest='radio_range', type=float, default=config['communication_range_metres'])
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600 or not 10 <= args.writer_seconds <= 600 or not 2 <= args.radio_range <= 30:
        raise ValueError('Motion seconds must be 10 to 600 and range must be 2 to 30 metres')
    from check_step3 import structural_check
    structural_check()
    extra_message_types()
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    scenario = {'completed': False, 'native_gazebo_runtime': True, 'phase': 'writer',
                'writer': {'completed': False}, 'executor': {'completed': False}}
    write_json(logs/'step3_actions.json', [])
    write_json(logs/'step3_result.json', scenario)
    connection = None
    try:
        writer, records = run_writer(args, scenario)
        print('HANDOFF: writer returned outside; starting the gray robot.', flush=True)
        config.update(motion_seconds=args.seconds, communication_range_metres=args.radio_range)
        plan, grid = plan_executor(config, records, writer)
        write_json(logs/'step3_route.json', plan)
        grid.save(logs/'step3_map.json')
        scenario['phase'] = 'executor'
        report = scenario['executor']
        manifest = json.loads((PACKAGE/'step3/scene.json').read_text())
        tracked = [b['beacon_name'] for b in records]+manifest['fire_ids']+manifest['balls'] \
            +[writer_config()['model_name']]
        connection = Step3Connection(config, tracked)
        def checkpoint(data):
            write_json(logs/'step3_actions.json', data.get('actions', []))
            write_json(logs/'step3_result.json', scenario)
        with (logs/'step3_trace.csv').open('w', newline='') as stream:
            trace = csv.writer(stream)
            trace.writerow(['wall_elapsed', 'sim_time', 'x', 'y', 'yaw', 'motion_elapsed', 'mode', 'event'])
            run_executor(connection, config, plan, grid, report, trace=trace, checkpoint=checkpoint)
        connection.verify_beacons(records)
        report['all_writer_beacons_preserved'] = True
        scenario.update(completed=True, phase='finished')
    except BaseException as error:
        scenario['reason'] = str(error) or type(error).__name__
        raise
    finally:
        if connection is not None:
            try:
                scenario['executor'].update(connection.diagnostics())
            finally:
                connection.close()
        write_json(logs/'step3_actions.json', scenario['executor'].get('actions', []))
        write_json(logs/'step3_result.json', scenario)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('Step 3 scenario stopped.', flush=True)
        sys.exit(130)
    except Exception as error:
        print('STEP 3 ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)

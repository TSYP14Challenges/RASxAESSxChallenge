#!/usr/bin/env python3
"""Move the visible Gazebo models through the complete route on a wall-time budget.

This is a kinematic scenario animation. The separate drive_step1.py uses wheel
physics. Every animation command is checked against actual Gazebo pose feedback.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from step1_control import wrap
from step1_feedback import Feedback
from step1_map import GridMap
from step1_pose_service import PoseServiceProcess
from step1_settings import load_config, motion_arguments, validate_motion
from step1_timeline import Timeline, model_poses

PACKAGE = Path(__file__).resolve().parents[1]


def transport_imports():
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.clock_pb2 import Clock
        return Node, Pose, Pose_V, Boolean, Clock
    except ImportError as error:
        raise RuntimeError('Install: sudo apt install python3-gz-transport13 python3-gz-msgs10\n'
                           f'Detail: {error}') from error


class AnimationConnection(Feedback):
    def __init__(self, config, worker_log=None):
        super().__init__(config)
        Node, self.Pose, self.Pose_V, self.Boolean, Clock = transport_imports()
        self.node = Node()
        self.service = f'/world/{config["world_name"]}/set_pose_vector'
        self.single_service = f'/world/{config["world_name"]}/set_pose'
        self.use_vector = True
        self.request_failures = self.request_retries = 0
        self.fallback_reason = self.last_failure = None
        self.callbacks = []
        for topic in self.topics:
            def callback(message, source=topic):
                self.receive(message, source)
            self.callbacks.append(callback)
            if not self.node.subscribe(self.Pose_V, topic, callback):
                raise RuntimeError('Could not subscribe to '+topic)
        if not self.node.subscribe(Clock, self.clock_topic, self.receive_clock):
            raise RuntimeError('Could not subscribe to '+self.clock_topic)
        self.requester = PoseServiceProcess(worker_log or PACKAGE/'logs/step1_pose_worker.log')

    @staticmethod
    def fill_pose(pose, values):
        name, x, y, z, qx, qy, qz, qw = values
        pose.name = name
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw

    def request_pose(self, service, message, message_type, model):
        # Repeating an absolute pose is safe even if a timed-out request was
        # applied. Never advance to a new target while recovering this one.
        for attempt, timeout in enumerate((2000, 5000)):
            if attempt:
                self.request_retries += 1
                print(f'Pose update retry: {self.last_failure}; '
                      f'allowing {timeout/1000:g} seconds for {service}.', flush=True)
                time.sleep(.05)
            try:
                result, response = self.requester.request(service, message, message_type, self.Boolean, timeout)
                if result and response is not None and response.data:
                    return True
                if not result:
                    reason = f'{model}: request failed or timed out ({timeout} ms)'
                elif response is None:
                    reason = f'{model}: missing Boolean reply'
                else:
                    reason = f'{model}: Gazebo rejected the update (data=false)'
            except (RuntimeError, OSError) as error:
                reason = f'{model}: transport error: {error}'
            self.request_failures += 1
            self.last_failure = reason
        return False

    def prepare_pose(self, pose, values):
        self.fill_pose(pose, values)

    def send(self, state):
        values = model_poses(state, self.name)
        if self.use_vector:
            message = self.Pose_V()
            for value in values:
                self.prepare_pose(message.pose.add(), value)
            if self.request_pose(self.service, message, self.Pose_V, 'robot and wheels'):
                return
            self.use_vector = False
            self.fallback_reason = self.last_failure
            print(f'Batch pose updates unavailable: {self.fallback_reason}.\n'
                  f'Pose updates: using {self.single_service} for the robot and both wheels.', flush=True)
        for value in values:
            message = self.Pose()
            self.prepare_pose(message, value)
            if not self.request_pose(self.single_service, message, self.Pose, message.name):
                raise RuntimeError(f'Could not update {message.name} through {self.single_service}: '
                                   f'{self.last_failure}. Batch service {self.service}: '
                                   f'{self.fallback_reason}. Inspect logs/step1_pose_worker.log '
                                   'and logs/gazebo-step1.log.')

    def diagnostics(self):
        return {'pose_service': self.service if self.use_vector else self.single_service,
                'pose_request_failures': self.request_failures,
                'pose_request_retries': self.request_retries,
                'pose_service_fallback_reason': self.fallback_reason,
                'last_pose_request_failure': self.last_failure,
                'pose_requests_isolated': True, 'pose_worker_pid': self.requester.pid}

    def close(self):
        self.requester.close()


def pose_matches(sample, commanded):
    return math.hypot(sample['x']-commanded['x'], sample['y']-commanded['y']) <= .025 \
        and abs(wrap(sample['yaw']-commanded['yaw'])) <= .035 \
        and abs(sample['z']-.005) <= .025


def main():
    config = load_config(PACKAGE)
    parser = argparse.ArgumentParser(description=__doc__)
    motion_arguments(parser, config)
    parser.add_argument('--seconds', type=float, default=config['preview_duration_seconds'],
                        help='Target real seconds for the complete robot tour (default: 90)')
    args = parser.parse_args()
    validate_motion(args)
    if not 10 <= args.seconds <= 600:
        raise ValueError('--seconds must be between 10 and 600')
    route = json.loads((PACKAGE/'step1/route.json').read_text())
    for key, file in [('world_sha256', PACKAGE/'worlds/warehouse.sdf'),
                      ('config_sha256', PACKAGE/'step1/config.json')]:
        if route[key] != hashlib.sha256(file.read_bytes()).hexdigest():
            raise RuntimeError('World/config changed. Run bash run.sh plan first.')
    timeline = Timeline(route['waypoints'], config, args.seconds, args.speed)
    grid = GridMap.load(PACKAGE/'step1/map.json')
    points = [(w['x'], w['y']) for w in route['waypoints']]
    if not all(grid.line_safe(a, b) for a, b in zip(points, points[1:])):
        raise RuntimeError('The route crosses a blocked map cell. Run plan and check first.')
    connection = AnimationConnection(config)
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    report = {'completed': False, 'motion_mode': 'kinematic_route_animation',
              'requested_real_seconds': args.seconds, 'route_metres': round(timeline.distance, 2),
              'aisle_sections': len(route['aisle_segments']), 'visited_sections': []}
    try:
        print('FAST TOUR: route animation; wheel physics is available with --physics.', flush=True)
        print('Pose requests: separate worker; clock and pose feedback stay in the controller.', flush=True)
        print('Waiting for the actual Gazebo robot pose and clock...', flush=True)
        waiting = time.monotonic()
        sample = connection.state()
        while sample is None or not sample['pose_fresh']:
            if time.monotonic()-waiting > 90:
                raise RuntimeError('Robot pose/clock missing. Inspect logs/gazebo-step1.log.')
            time.sleep(.03)
            sample = connection.state()
        initial = timeline.sample(0.0)
        if not pose_matches(sample, initial):
            raise RuntimeError('The animated robot is not at the outside start. Restart the complete tour.')
        connection.send(initial)
        print(f'Tour: {len(route["aisle_segments"])} sections, {timeline.distance:.1f} m; '
              f'{args.seconds:g}-second target, {timeline.linear_speed:.2f} m/s translation. '
              'Timing uses real seconds.', flush=True)
        first_wall = last_wall = time.monotonic()
        clock_wall, previous_clock = first_wall, sample['sim_time']
        first_sim = previous_clock
        budget, progress, issued = -args.start_delay, 0.0, initial
        issued_wall, pose_wait, arrivals = first_wall, None, 0
        travelled, previous_xy = 0.0, (sample['x'], sample['y'])
        frames, last_status, active_name = 0, first_wall, ''
        with (logs/'step1_trace.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time', 'x', 'y', 'z', 'yaw', 'linear_command', 'angular_command',
                             'waypoint', 'mode', 'wall_elapsed', 'route_elapsed'])
            while True:
                frame_start = time.monotonic()
                sample = connection.state()
                now = time.monotonic()
                delta, last_wall = now-last_wall, now
                if sample is None or sample['time_reset']:
                    raise RuntimeError('The simulation reset. Restart the complete tour.')
                if sample['sim_time'] > previous_clock:
                    clock_wall, previous_clock = now, sample['sim_time']
                blocked = now-clock_wall > .5 or not sample['pose_fresh']
                if blocked:
                    if pose_wait is None:
                        pose_wait = now
                        print('Waiting for Play or recent Gazebo pose feedback; animation held.', flush=True)
                    if now-clock_wall <= .5 and now-pose_wait > 10:
                        raise RuntimeError('Clock advances but robot pose is stale for 10 seconds.')
                    time.sleep(.02)
                    continue
                pose_wait = None
                budget += delta
                if not pose_matches(sample, issued):
                    if now-issued_wall > 10:
                        raise RuntimeError('Gazebo accepted a pose command but the visible robot '
                                           'did not reach it. Animation stopped; inspect the Gazebo log.')
                    time.sleep(.005)
                    continue
                travelled += math.dist(previous_xy, (sample['x'], sample['y']))
                if not grid.line_safe(previous_xy, (sample['x'], sample['y'])):
                    raise RuntimeError('Actual robot pose left the safe corridor; animation stopped.')
                previous_xy = (sample['x'], sample['y'])
                while arrivals < len(timeline.arrivals) and \
                        timeline.arrivals[arrivals][0] <= progress+1e-9:
                    _, _, kind, name = timeline.arrivals[arrivals]
                    if kind == 'aisle':
                        report['visited_sections'].append(name)
                        print('Visited '+name, flush=True)
                    arrivals += 1
                if progress >= timeline.duration-1e-9:
                    expected = {s['name'] for s in route['aisle_segments']}
                    if set(report['visited_sections']) != expected:
                        raise RuntimeError('The animated tour is missing an aisle section.')
                    report.update(completed=True, elapsed_wall_seconds=round(now-first_wall, 2),
                                  simulation_seconds=round(sample['sim_time']-first_sim, 2),
                                  travelled_metres=round(travelled, 2),
                                  finish_error_metres=round(math.dist(previous_xy, config['start'][:2]), 5),
                                  frames_applied=frames)
                    print('COMPLETE: all 15 sections visited; returned outside and stopped.', flush=True)
                    print(f'Real elapsed time: {now-first_wall:.1f} seconds.', flush=True)
                    break
                following = timeline.advance(progress, max(0.0, min(timeline.duration, budget)-progress))
                if following > progress:
                    progress = following
                    issued = timeline.sample(progress)
                    if not grid.safe(issued['x'], issued['y']):
                        raise RuntimeError('Animation target is outside the safe corridor.')
                    connection.send(issued)
                    issued_wall = time.monotonic()
                    frames += 1
                    name = route['waypoints'][issued['index']]['name']
                    if name != active_name:
                        print(name, flush=True)
                        active_name = name
                    writer.writerow([sample['sim_time'], sample['x'], sample['y'], sample['z'],
                                     sample['yaw'], issued['linear'], issued['angular'], issued['index'],
                                     issued['mode'], round(now-first_wall, 4), round(progress, 4)])
                    stream.flush()
                if now-last_status >= 10:
                    rate = (sample['sim_time']-first_sim)/max(.001, now-first_wall)
                    print(f'  {progress:.1f}/{timeline.duration:.0f} route seconds | '
                          f'{len(report["visited_sections"])}/15 sections | '
                          f'Gazebo clock {rate:.2f}x real time', flush=True)
                    last_status = now
                time.sleep(max(.001, 1/config['preview_frame_rate']-(time.monotonic()-frame_start)))
    except KeyboardInterrupt:
        report['reason'] = 'Interrupted by the user'
        print('\nAnimation stopped.', flush=True)
    except Exception as error:
        report['reason'] = str(error)
        raise
    finally:
        # The gravity-free models have no continuing velocity command; they hold
        # the last confirmed pose when this loop ends or the user interrupts it.
        try:
            report.update(connection.diagnostics())
            (logs/'step1_result.json').write_text(json.dumps(report, indent=2)+'\n')
        finally:
            connection.close()
    return 0 if report['completed'] else 130


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('FAST TOUR ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)

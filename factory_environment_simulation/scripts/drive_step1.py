#!/usr/bin/env python3
"""Drive the physical Gazebo wheels using world-pose feedback and the saved safe tour."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from step1_control import Follower, wrap
from step1_feedback import Feedback
from step1_map import GridMap
from step1_settings import load_config, motion_arguments, validate_motion

PACKAGE = Path(__file__).resolve().parents[1]


def transport_imports():
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.twist_pb2 import Twist
        from gz.msgs10.clock_pb2 import Clock
        return Node, Pose_V, Twist, Clock
    except ImportError as error:
        raise RuntimeError('Gazebo Python bindings are missing for /usr/bin/python3.\n'
                           'Install: sudo apt install python3-gz-transport13 python3-gz-msgs10\n'
                           f'Detail: {error}') from error


class RobotConnection(Feedback):
    def __init__(self, config):
        super().__init__(config)
        Node, Pose_V, self.Twist, Clock = transport_imports()
        self.node = Node()
        self.publisher = self.node.advertise('/step1/cmd_vel', self.Twist)
        self.callbacks = []
        for topic in self.topics:
            def callback(message, source=topic):
                self.receive(message, source)
            self.callbacks.append(callback)
            if not self.node.subscribe(Pose_V, topic, callback):
                raise RuntimeError('Could not subscribe to '+topic)
        if not self.node.subscribe(Clock, self.clock_topic, self.receive_clock):
            raise RuntimeError('Could not subscribe to '+self.clock_topic)
        if not self.publisher:
            raise RuntimeError('Could not advertise /step1/cmd_vel')

    def send(self, linear=0.0, angular=0.0):
        message = self.Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        if not self.publisher.publish(message):
            raise RuntimeError('Publishing the wheel command failed')

    def stop(self):
        for _ in range(12):
            try:
                self.send()
            except Exception:
                pass
            time.sleep(0.04)


def main():
    config_path, world_path = PACKAGE/'step1/config.json', PACKAGE/'worlds/warehouse.sdf'
    config = load_config(PACKAGE)
    parser = argparse.ArgumentParser(description=__doc__)
    motion_arguments(parser, config)
    args = parser.parse_args()
    validate_motion(args)
    route = json.loads((PACKAGE/'step1/route.json').read_text())
    for name, file in [('world_sha256', world_path), ('config_sha256', config_path)]:
        if route[name] != hashlib.sha256(file.read_bytes()).hexdigest():
            raise RuntimeError('World/config changed. Run: /usr/bin/python3 scripts/build_step1.py')
    if args.speed is not None:
        config['linear_speed'] = args.speed
    grid = GridMap.load(PACKAGE/'step1/map.json')
    follower = Follower(route['waypoints'], config, grid)
    robot = RobotConnection(config)
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    report = {'completed': False, 'route_metres': route['distance_metres'],
              'aisle_sections': len(route['aisle_segments'])}
    try:
        print('STEP 1: waiting for the Gazebo robot pose and simulation clock...', flush=True)
        wait_start = time.monotonic()
        sample = None
        while sample is None:
            robot.send()
            sample = robot.state()
            if time.monotonic()-wait_start > 90:
                raise RuntimeError('Missing robot pose or simulation clock after 90 seconds. '
                                   'Check logs/gazebo-step1.log and ensure the simulation is playing. '
                                   'Expected clock: '+robot.clock_topic)
            time.sleep(0.03)
        if math.hypot(sample['x']-config['start'][0], sample['y']-config['start'][1]) > 0.8:
            raise RuntimeError('Robot is not at the outside starting point. Restart this demo.')
        print('Connected to the actual model pose through '+sample['topic'], flush=True)
        print('Simulation time from '+robot.clock_topic, flush=True)
        print(f'Tour: {len(route["aisle_segments"])} aisle sections, {route["distance_metres"]:.1f} m. '
              f'Speed: {config["linear_speed"]:.2f} m/s.', flush=True)
        print(f'Waiting {max(0,args.start_delay):g} simulated seconds before entry.', flush=True)
        report.update(clock_topic=robot.clock_topic, initial_pose_topic=sample['topic'],
                      start_sim_time=sample['sim_time'])
        first_sim, previous_sim = sample['sim_time'], -1.0
        first_wall = time.monotonic()
        last_status_wall = first_wall
        progress_wall, last_status, last_trace = time.monotonic(), -99.0, -99.0
        active_name, paused, stall_anchor = '', False, sample
        entering, pose_wait_start = False, None
        accumulated_distance, previous_xy = 0.0, (sample['x'], sample['y'])
        with (logs/'step1_trace.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['sim_time', 'x', 'y', 'z', 'yaw', 'linear_command',
                             'angular_command', 'waypoint', 'mode'])
            while True:
                sample = robot.state()
                now = time.monotonic()
                sim = sample['sim_time']
                if sample['time_reset'] or sim < previous_sim-0.1:
                    raise RuntimeError('Simulation time reset. Restart the complete demo.')
                if sim <= previous_sim:
                    if now-progress_wall > 0.8:
                        robot.send()
                        follower.last_speed = 0.0
                        if not paused:
                            print('Waiting: Gazebo clock is not advancing; robot held still. '
                                  'Press Play in Gazebo if paused.', flush=True)
                            paused = True
                    time.sleep(0.015)
                    continue
                dt = min(0.1, sim-previous_sim) if previous_sim >= 0 else 1/30
                previous_sim, progress_wall, paused = sim, now, False
                if not sample['pose_fresh']:
                    robot.send()
                    follower.last_speed = 0.0
                    if pose_wait_start is None:
                        pose_wait_start = now
                        print('Waiting: clock advances but robot pose is stale; wheels stopped.', flush=True)
                    if now-pose_wait_start > 10:
                        raise RuntimeError('No recent robot pose for 10 seconds while the clock advances. '
                                           'Inspect the PosePublisher plugin in logs/gazebo-step1.log.')
                    time.sleep(0.015)
                    continue
                pose_wait_start = None
                x, y, yaw = sample['x'], sample['y'], sample['yaw']
                if sample['z'] < -0.10 or sample['z'] > 0.25:
                    raise RuntimeError('Unexpected robot height. Stopped; inspect wheel/floor contact.')
                accumulated_distance += math.dist(previous_xy, (x, y))
                previous_xy = (x, y)
                if sim-first_sim < max(0, args.start_delay):
                    linear, angular, follower.mode = 0.0, 0.0, 'outside: waiting'
                else:
                    if not entering:
                        print('Entering the factory: sending wheel commands on /step1/cmd_vel.', flush=True)
                        entering = True
                    linear, angular = follower.command(x, y, yaw, dt)
                robot.send(linear, angular)
                if sim-last_trace >= 0.18 or follower.finished:
                    writer.writerow([round(sim, 4), round(x, 6), round(y, 6), round(sample['z'], 6),
                                     round(yaw, 6), round(linear, 6), round(angular, 6),
                                     follower.index, follower.mode])
                    stream.flush()
                    last_trace = sim
                name = route['waypoints'][min(follower.index, len(route['waypoints'])-1)]['name']
                if entering and name != active_name:
                    print(name, flush=True)
                    active_name = name
                if (sim-last_status > 10 or now-last_status_wall > 10) and sim-first_sim >= args.start_delay:
                    clock_rate = (sim-first_sim)/max(.001, now-first_wall)
                    print(f'  {follower.index}/{len(route["waypoints"])-1} | '
                          f'({x:.1f}, {y:.1f}) | {follower.mode} | '
                          f'command {linear:.2f} m/s | Gazebo clock {clock_rate:.2f}x real time', flush=True)
                    last_status, last_status_wall = sim, now
                movement = math.hypot(x-stall_anchor['x'], y-stall_anchor['y'])
                turning = abs(wrap(yaw-stall_anchor['yaw']))
                if movement > 0.025 or turning > 0.055 or (linear < 0.04 and abs(angular) < 0.08):
                    stall_anchor = sample
                elif sim-stall_anchor['sim_time'] > 8:
                    raise RuntimeError('Wheel commands produced no model movement for 8 simulated seconds. '
                                       'Inspect logs/gazebo-step1.log and the robot contacts.')
                if follower.finished:
                    report.update(completed=True, simulation_seconds=round(sim-first_sim, 2),
                                  elapsed_wall_seconds=round(now-first_wall, 2),
                                  travelled_metres=round(accumulated_distance, 2),
                                  finish=[round(x, 5), round(y, 5)],
                                  finish_error_metres=round(math.dist((x, y), config['start'][:2]), 5))
                    print('COMPLETE: all planned aisle sections visited; returned outside and stopped.', flush=True)
                    print(f'Tour duration: {sim-first_sim:.1f} simulated seconds; '
                          f'{now-first_wall:.1f} real seconds.', flush=True)
                    break
                time.sleep(0.002)
    except KeyboardInterrupt:
        report['reason'] = 'Interrupted by the user'
        print('\nStopped the robot.', flush=True)
    except Exception as error:
        report['reason'] = str(error)
        raise
    finally:
        robot.stop()
        (logs/'step1_result.json').write_text(json.dumps(report, indent=2)+'\n')
    return 0 if report['completed'] else 130


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('STEP 1 ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)

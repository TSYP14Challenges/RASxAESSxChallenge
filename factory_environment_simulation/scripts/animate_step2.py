#!/usr/bin/env python3
"""Run the writer's known-layout event mission using confirmed Gazebo poses."""
import argparse
import csv
import json
import math
import os
import sys
import time

# Select this before importing any Gazebo / protobuf module. Gazebo Transport
# exchanges serialized bytes, so Python messages need no shared native factory.
# The native factory can lose the nested Pose registration after EntityFactory
# is imported before the transport extension.
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

from animate_step1 import AnimationConnection, transport_imports
from build_step2 import beacon_sdf
from step1_feedback import Feedback
from step1_map import GridMap, read_geometry
from step1_pose_service import PoseServiceProcess
from step1_control import wrap
from step2_feedback_service import FeedbackProcess
from step2_events import MissionEvents, PACKAGE, load_config
from step2_timeline import MissionTimeline


def pose_matches(sample, commanded):
    """Confirm this kinematic target, rather than accepting the previous frame."""
    return math.hypot(sample['x']-commanded['x'], sample['y']-commanded['y']) <= .002 \
        and abs(wrap(sample['yaw']-commanded['yaw'])) <= .004 \
        and abs(sample['z']-.005) <= .002


class Step2Connection(AnimationConnection):
    def __init__(self, config, model_names, worker_prefix='step2'):
        self.EntityFactory = creation_message_type()
        self.config = config
        self.model_names, self.model_samples = set(model_names), {}
        self.create_service = f'/world/{config["world_name"]}/create'
        self.created_models = set()
        self.create_failures = self.create_retries = 0
        self.last_create_failure = None
        Feedback.__init__(self, {**config, 'prefer_newest_pose': True})
        _, self.Pose, self.Pose_V, self.Boolean, _ = transport_imports()
        # Sim 8.11 provides these endpoints. Unlike the async endpoint's queue
        # acknowledgement, their Boolean reports whether the command executed.
        # Physical application is still verified from pose feedback afterwards.
        self.service = f'/world/{config["world_name"]}/set_pose_vector/blocking'
        self.single_service = f'/world/{config["world_name"]}/set_pose/blocking'
        self.use_vector = True
        self.request_failures = self.request_retries = 0
        self.fallback_reason = self.last_failure = None
        self.motion_recoveries, self.last_motion_mismatch = 0, None
        self.command_pose_time = None
        self.entity_ids = {}
        self.tracked_models = self.model_names | {self.name, self.name+'_left_wheel', self.name+'_right_wheel'}
        self.requester = self.feedback_process = None
        try:
            self.requester = PoseServiceProcess(PACKAGE/f'logs/{worker_prefix}_pose_worker.log')
            self.feedback_process = FeedbackProcess(
                config, self.tracked_models, self.receive, self.receive_clock,
                PACKAGE/f'logs/{worker_prefix}_feedback_worker.log')
        except BaseException:
            self.close()
            raise

    def receive(self, message, source):
        super().receive(message, source)
        for pose in message.pose:
            name = pose.name.split('::')[-1]
            entity_id = int(getattr(pose, 'id', 0))
            if entity_id and name in self.tracked_models:
                with self.lock:
                    self.entity_ids[name] = entity_id
            if name not in self.model_names:
                continue
            p = pose.position
            if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                with self.lock:
                    self.model_samples[name] = (p.x, p.y, p.z, time.monotonic())

    def state(self):
        if self.feedback_process is not None:
            self.feedback_process.check()
        return super().state()

    def prepare_pose(self, pose, values):
        super().prepare_pose(pose, values)
        with self.lock:
            entity_id = self.entity_ids.get(values[0])
        if entity_id:
            pose.id = entity_id

    def send(self, state):
        sample = self.state()
        # Gazebo retains WorldPoseCmd for an extra physics iteration. Wait
        # until feedback has passed both iterations before issuing a new
        # target, including tiny movements that fit within spatial tolerance.
        self.command_pose_time = None if sample is None else \
            sample['sim_time']+2*self.config['preview_physics_step']
        super().send(state)

    def motion_confirmed(self, sample, commanded):
        if not pose_matches(sample, commanded):
            return False
        if self.command_pose_time is None:
            return True
        stamp = sample.get('pose_time')
        return stamp is not None and stamp+1e-8 >= self.command_pose_time

    def motion_snapshot(self, commanded, sample):
        with self.lock:
            now = time.monotonic()
            sources = [{**s, 'receipt_age_seconds': round(now-s['received'], 3)}
                       for s in self.samples.values()]
            ids = dict(self.entity_ids)
        return {'commanded': {k: commanded[k] for k in ('x', 'y', 'yaw')},
                'observed': {k: sample[k] for k in ('x', 'y', 'z', 'yaw', 'sim_time')},
                'required_pose_time': self.command_pose_time,
                'entity_ids': ids, 'pose_sources': sources}

    def recover_motion(self, commanded, sample):
        self.motion_recoveries += 1
        self.last_motion_mismatch = self.motion_snapshot(commanded, sample)
        if self.use_vector:
            self.use_vector = False
            self.fallback_reason = 'Batch command acknowledged but actual robot pose did not match'
        detail = self.last_motion_mismatch
        if getattr(self.requester, 'log', None) is not None:
            self.requester.log.write((json.dumps({'motion_recovery': self.motion_recoveries, **detail})+'\n')
                                     .encode('utf-8'))
        print(f'Movement recovery {self.motion_recoveries}: observed '
              f'({sample["x"]:.3f}, {sample["y"]:.3f}); target '
              f'({commanded["x"]:.3f}, {commanded["y"]:.3f}). '
              'Reapplying the same target to the robot and wheels through set_pose/blocking.', flush=True)
        self.send(commanded)

    def close(self):
        # End the callback owner before this interpreter can finalize. The
        # controller contains only a pure-Python reader thread and message data.
        try:
            if self.feedback_process is not None:
                self.feedback_process.close()
        finally:
            if self.requester is not None:
                self.requester.close()

    def applied(self, values):
        with self.lock:
            now = time.monotonic()
            return all(name in self.model_samples and
                       math.dist(self.model_samples[name][:2], (x, y)) <= .018 and
                       abs(self.model_samples[name][2]-z) <= .0005 and
                       now-self.model_samples[name][3] <= 1.0
                       for name, x, y, z, *_ in values)

    def create_and_confirm(self, name, sdf, position):
        values = [(name, *position)]
        if self.applied(values):
            self.created_models.add(name)
            return
        message = self.EntityFactory()
        message.name, message.sdf = name, sdf
        # A timeout may happen after creation. A retry must retain the same
        # name and must not create a second, automatically renamed model.
        message.allow_renaming = False
        for attempt, timeout in enumerate((2000, 5000)):
            if attempt:
                self.create_retries += 1
                print(f'Beacon creation retry: {name}; keeping its original name.', flush=True)
            accepted = False
            try:
                result, response = self.requester.request(
                    self.create_service, message, self.EntityFactory, self.Boolean, timeout)
                accepted = bool(result and response is not None and response.data)
                if not accepted:
                    self.last_create_failure = f'{name}: creation failed or timed out ({timeout} ms)'
            except (RuntimeError, OSError) as error:
                self.last_create_failure = f'{name}: transport error: {error}'
            if not accepted:
                self.create_failures += 1
            deadline = time.monotonic()+(10.0 if accepted else 1.0)
            while not self.applied(values):
                if time.monotonic() >= deadline:
                    break
                time.sleep(.02)
            if self.applied(values):
                self.created_models.add(name)
                return
            if accepted:
                raise RuntimeError('Gazebo accepted the drop but beacon/cable pose was not confirmed: '+name)
        raise RuntimeError('Could not create and confirm '+name+': '+str(self.last_create_failure))

    def place(self, event):
        if event['kind'] == 'gateway':
            # The fixed tile and wire are already in the validated world.
            # The entrance pause activates the uplink; no spawn is needed.
            self.created_models.add(event['beacon_name'])
            return
        self.create_and_confirm(event['beacon_name'],
                                beacon_sdf(event['beacon_name'], event['kind'],
                                           event['position'], self.config), event['position'])

    def diagnostics(self):
        return {**super().diagnostics(), 'beacon_deployment': 'fixed_entrance_and_create_event_tiles',
                'protobuf_implementation': 'python',
                'beacon_create_service': self.create_service,
                'beacon_create_request_failures': self.create_failures,
                'beacon_create_request_retries': self.create_retries,
                'last_beacon_create_failure': self.last_create_failure,
                'created_beacon_models': sorted(self.created_models),
                'entrance_assembly': {'model': 'step2_entrance_blue_beacon',
                                     'position': self.config['entrance_beacon_position'],
                                     'cable_in_same_model': True, 'fixed': True,
                                     'present_at_startup': True},
                'pose_command_reply_mode': 'blocking_execution_result',
                'feedback_subscriptions_isolated': True,
                'feedback_worker_pid': self.feedback_process.pid,
                'full_scene_pose_subscription': False,
                'feedback_pose_topics': getattr(self.feedback_process, 'pose_topics', []),
                'movement_position_tolerance_metres': .002,
                'movement_sync_physics_steps': 2,
                'motion_application_retries': self.motion_recoveries,
                'last_motion_mismatch': self.last_motion_mismatch}


def creation_message_type():
    # Keep the working Step 1 order: transport extension, pose messages, factory.
    _, Pose, Pose_V, Boolean, Clock = transport_imports()
    try:
        from gz.msgs10.entity_factory_pb2 import EntityFactory
    except ImportError as error:
        raise RuntimeError('Install: sudo apt install python3-gz-msgs10\n'+str(error)) from error
    try:
        from google.protobuf.internal import api_implementation
        if api_implementation.Type() != 'python':
            raise RuntimeError('The native protobuf parser was already loaded. '
                               'Restart using bash run.sh demo in a new Python process.')
        validate_message_types(Pose, Pose_V, Boolean, Clock, EntityFactory)
    except (TypeError, AttributeError, ValueError) as error:
        raise RuntimeError('Gazebo Python pose-message check failed before starting the mission: '
                           +str(error)) from error
    return EntityFactory


def validate_message_types(Pose, Pose_V, Boolean, Clock, EntityFactory):
    """Decode real nested fields before starting subscription threads or Gazebo."""
    vector = Pose_V()
    vector.header.stamp.sec, vector.header.stamp.nsec = 12, 123
    expected = [('robot_probe', 0., -14., .005), ('beacon_probe', -.45, -11., .016),
                ('cable_probe', 0., 0., 0.)]
    for name, x, y, z in expected:
        p = vector.pose.add()
        p.name = name
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.
        p.header.stamp.sec, p.header.stamp.nsec = 12, 123
    decoded = Pose_V()
    decoded.ParseFromString(vector.SerializeToString())
    observed = [(p.name, p.position.x, p.position.y, p.position.z) for p in decoded.pose]
    if observed != expected or decoded.header.stamp.sec != 12:
        raise ValueError('Pose_V nested fields do not match the serialized poses')
    for p in decoded.pose:
        single = Pose()
        single.ParseFromString(p.SerializeToString())
        if single.orientation.w != 1. or single.header.stamp.nsec != 123:
            raise ValueError('Pose orientation or timestamp could not be decoded')
    clock = Clock()
    clock.sim.sec, clock.sim.nsec = 12, 123
    clock_copy = Clock()
    clock_copy.ParseFromString(clock.SerializeToString())
    if clock_copy.sim.sec != 12 or clock_copy.sim.nsec != 123:
        raise ValueError('Simulation clock could not be decoded')
    factory = EntityFactory()
    factory.name, factory.sdf = 'registration_probe', '<sdf version="1.9"/>\n'
    factory.allow_renaming = False
    factory.pose.position.z, factory.pose.orientation.w = .016, 1.
    factory_copy = EntityFactory()
    factory_copy.ParseFromString(factory.SerializeToString())
    if factory_copy.sdf != factory.sdf or factory_copy.allow_renaming \
            or factory_copy.pose.position.z != .016:
        raise ValueError('Beacon creation message could not be decoded')
    response = Boolean()
    response.data = True
    response_copy = Boolean()
    response_copy.ParseFromString(response.SerializeToString())
    if not response_copy.data:
        raise ValueError('Service reply could not be decoded')
    return True


def run_mission(connection, config, route, grid, mission, seconds, report,
                trace=None, clock=time, checkpoint=None):
    timeline = MissionTimeline(route['waypoints'], config, seconds)
    print('STEP 2: writer robot; known-position event detection and permanent beacons.', flush=True)
    print('Transport: small pose streams; each movement waits for fresh applied-pose feedback.', flush=True)
    print('Beacons: created at their floor positions when dropped; no underground staging.', flush=True)
    print('Waiting for the actual Gazebo robot pose and clock...', flush=True)
    waiting = clock.monotonic()
    while True:
        sample = connection.state()
        if sample and sample['pose_fresh']:
            break
        if clock.monotonic()-waiting > 90:
            raise RuntimeError('Robot pose/clock missing. Inspect logs/gazebo-step2.log.')
        clock.sleep(.03)
    issued = timeline.sample(0.0)
    if not pose_matches(sample, issued):
        raise RuntimeError('Writer is not at the outside start. Restart the complete mission.')
    connection.send(issued)
    first_wall = last_wall = clock.monotonic()
    first_sim = previous_clock = sample['sim_time']
    clock_wall, issued_wall = first_wall, first_wall
    recovery_wall, recovery_attempts = first_wall, 0
    budget, progress, pending, pause_elapsed = -config['start_delay'], 0.0, None, 0.0
    arrivals, frames, last_status, stalled = 0, 0, first_wall, None
    previous_xy, travelled = (sample['x'], sample['y']), 0.0
    print(f'Tour: 15 sections, {timeline.distance:.1f} m; {seconds:g} motion seconds '
          f'plus {config["drop_pause_seconds"]:g} second per drop.\n'
          f'Detection: victim 0.30 m, fire 1.00 m from the robot edge; '
          f'nearest interior beacon range {config["communication_range_metres"]:g} m.', flush=True)
    report.update(requested_motion_seconds=seconds, route_metres=timeline.distance,
                  communication_range_metres=config['communication_range_metres'],
                  visited_sections=[], beacons=[], communication=mission.network.snapshot())
    while True:
        frame_start = clock.monotonic()
        sample, now = connection.state(), clock.monotonic()
        delta, last_wall = now-last_wall, now
        if sample is None or sample['time_reset']:
            raise RuntimeError('The simulation reset. Restart the mission.')
        if sample['sim_time'] > previous_clock:
            clock_wall, previous_clock = now, sample['sim_time']
        if now-clock_wall > .5 or not sample['pose_fresh']:
            if stalled is None:
                stalled = now
                print('Waiting for Play or recent pose feedback; mission held.', flush=True)
            if now-clock_wall <= .5 and now-stalled > 10:
                raise RuntimeError('Clock advances but writer pose is stale for 10 seconds')
            clock.sleep(.02)
            continue
        stalled = None
        # Count active motion time even while waiting for the last movement to
        # appear in feedback. Exclude the explicit beacon pause and Play/stale
        # waits above. Previously every feedback wait erased this time, creating
        # centimetre-sized commands and making a 90 s tour crawl or stall.
        if pending is None:
            budget += delta
        if not connection.motion_confirmed(sample, issued):
            if now-issued_wall > 10:
                detail = connection.motion_snapshot(issued, sample)
                connection.last_motion_mismatch = detail
                raise RuntimeError('Writer pose did not match after movement recovery. '
                                   f'Observed ({sample["x"]:.3f}, {sample["y"]:.3f}, yaw {sample["yaw"]:.3f}); '
                                   f'target ({issued["x"]:.3f}, {issued["y"]:.3f}, yaw {issued["yaw"]:.3f}). '
                                   'Inspect logs/step2_result.json and logs/step2_pose_worker.log.')
            if recovery_attempts < 3 and now-recovery_wall >= (1.0 if not recovery_attempts else 2.0):
                connection.recover_motion(issued, sample)
                recovery_attempts += 1
                recovery_wall = last_wall = clock.monotonic()
            clock.sleep(.005)
            continue
        actual_xy = sample['x'], sample['y']
        if not grid.line_safe(previous_xy, actual_xy):
            raise RuntimeError('Writer left the safe corridor')
        travelled += math.dist(previous_xy, actual_xy)
        previous_xy = actual_xy
        while arrivals < len(timeline.arrivals) and timeline.arrivals[arrivals][0] <= progress+1e-9:
            _, _, kind, name = timeline.arrivals[arrivals]
            if kind == 'aisle':
                report['visited_sections'].append(name)
                print('Visited '+name, flush=True)
            arrivals += 1
        if pending is None:
            pending = mission.candidate(sample)
            if pending:
                pause_elapsed = 0.0
                kind_label = 'ENTRANCE BLUE' if pending['kind'] == 'gateway' else pending['kind'].upper()
                action = 'activating the entrance link' if pending['kind'] == 'gateway' else 'the drop'
                print(f'DETECT {kind_label}: {pending["id"]}; '
                      f'distance {pending["distance"]:.2f} m. Pausing before {action}...', flush=True)
                if trace:
                    trace.writerow([now-first_wall, sample['sim_time'], *actual_xy, sample['yaw'],
                                    progress, 'pause_'+pending['kind'], pending['id']])
                clock.sleep(1/config['preview_frame_rate'])
                continue
        if pending:
            pause_elapsed += delta
            if pause_elapsed+1e-9 < config['drop_pause_seconds']:
                clock.sleep(1/config['preview_frame_rate'])
                continue
            connection.place(pending)
            after = connection.state()
            if not after or not after['pose_fresh'] or not connection.motion_confirmed(after, issued):
                raise RuntimeError('Writer pose changed or became stale during the beacon drop')
            record = mission.commit(pending, sample, clock.monotonic()-first_wall, pause_elapsed)
            report['beacons'] = mission.records
            report['communication'] = mission.network.snapshot()
            if checkpoint:
                checkpoint(report)
            kind_label = 'ENTRANCE BLUE' if pending['kind'] == 'gateway' else pending['kind'].upper()
            action = 'ACTIVATE' if pending['kind'] == 'gateway' else 'DROP'
            print(f'{action} {kind_label}: {pending["beacon_name"]}; '
                  f'{len(mission.beacons)} beacons remain on the floor.', flush=True)
            if pending['kind'] == 'gateway':
                print('ENTRANCE: blue tile and connected wire already present in the scene, '
                      f'1 m inside the doorway at {record["position"]}.', flush=True)
                print('WIRED LINK UP: entrance blue beacon -> outside gateway.', flush=True)
            print(f'GATEWAY RECEIVED: {record["id"]}; '
                  +' -> '.join(record['data_route'])+' (last hop: cable).', flush=True)
            if trace:
                trace.writerow([clock.monotonic()-first_wall, sample['sim_time'], *actual_xy,
                                sample['yaw'], progress, 'drop_'+pending['kind'], pending['id']])
            pending, last_wall = None, clock.monotonic()
            clock.sleep(.001)
            continue
        if progress >= timeline.duration-1e-9:
            mission.require_complete()
            report['communication'] = mission.network.snapshot()
            expected = {s['name'] for s in route['aisle_segments']}
            if set(report['visited_sections']) != expected or len(report['visited_sections']) != len(expected):
                raise RuntimeError('The mission is missing or repeating an aisle section')
            report.update(completed=True, elapsed_wall_seconds=round(now-first_wall, 3),
                          simulation_seconds=round(sample['sim_time']-first_sim, 3),
                          travelled_metres=round(travelled, 3), frames_applied=frames,
                          finish_error_metres=math.dist(actual_xy, config['start'][:2]),
                          range_beacons=mission.range_count, beacons=mission.records)
            print(f'COMPLETE: all fires and victims marked, {mission.range_count} range beacons, '
                  f'all 15 aisles visited; writer returned outside and stopped.\n'
                  f'{len(mission.beacons)} beacons stay in place. Real elapsed: {now-first_wall:.1f} s.', flush=True)
            return report
        following = timeline.advance(progress, max(0.0, min(timeline.duration, budget)-progress))
        if following > progress:
            progress = following
            issued = timeline.sample(progress)
            if not grid.safe(issued['x'], issued['y']):
                raise RuntimeError('Unsafe animation target')
            connection.send(issued)
            issued_wall = recovery_wall = clock.monotonic()
            recovery_attempts = 0
            frames += 1
            if trace:
                trace.writerow([now-first_wall, sample['sim_time'], *actual_xy,
                                sample['yaw'], progress, issued['mode'], ''])
        if now-last_status >= 10:
            print(f'  {progress:.1f}/{timeline.duration:.0f} motion seconds | '
                  f'{len(report["visited_sections"])}/15 aisles | {len(mission.beacons)} beacons', flush=True)
            last_status = now
        clock.sleep(max(.001, 1/config['preview_frame_rate']-(clock.monotonic()-frame_start)))


def main():
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=config['motion_seconds'])
    parser.add_argument('--range', dest='radio_range', type=float, default=config['communication_range_metres'])
    parser.add_argument('--start-delay', type=float, default=config['start_delay'])
    args = parser.parse_args()
    if not 2 <= args.radio_range <= 30 or args.start_delay < 0:
        raise ValueError('Range must be 2 to 30 metres; start delay must be nonnegative')
    config.update(communication_range_metres=args.radio_range, start_delay=args.start_delay)
    from check_step2 import structural_check
    route, grid = structural_check(config)
    floors, obstacles, fires = read_geometry(PACKAGE/'worlds/warehouse.sdf', config)
    mission = MissionEvents(config, route['events'], floors, obstacles, fires)
    connection = Step2Connection(config, route['pose_models'])
    logs = PACKAGE/'logs'
    logs.mkdir(exist_ok=True)
    report = {'completed': False, 'motion_mode': 'kinematic_event_mission',
              'detection_mode': 'known_positions_edge_distance',
              'native_gazebo_runtime': True, 'beacons': [],
              'communication': mission.network.snapshot()}
    def checkpoint(data):
        (logs/'step2_events.json').write_text(json.dumps(data['beacons'], indent=2)+'\n')
        (logs/'gateway_inbox.json').write_text(json.dumps(mission.network.snapshot(), indent=2)+'\n')
        (logs/'step2_result.json').write_text(json.dumps(data, indent=2)+'\n')
    try:
        checkpoint(report)
        with (logs/'step2_trace.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['wall_elapsed', 'sim_time', 'x', 'y', 'yaw', 'motion_elapsed', 'mode', 'event'])
            run_mission(connection, config, route, grid, mission, args.seconds, report,
                        trace=writer, checkpoint=checkpoint)
    except KeyboardInterrupt:
        report['reason'] = 'Interrupted by the user'
        print('\nMission stopped; placed beacons stay in position.', flush=True)
    except Exception as error:
        report['reason'] = str(error)
        raise
    finally:
        try:
            report['beacons'] = mission.records
            report['communication'] = mission.network.snapshot()
            report.update(connection.diagnostics())
            checkpoint(report)
        finally:
            connection.close()
    return 0 if report['completed'] else 130


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('STEP 2 ERROR:', error, file=sys.stderr, flush=True)
        sys.exit(1)

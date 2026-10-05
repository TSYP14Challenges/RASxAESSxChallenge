#!/usr/bin/env python3
"""Exercise movement recovery and actual feedback subprocess lifecycle offline."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

from animate_step2 import Step2Connection, pose_matches
from check_step1_pose_service import MESSAGE, Probe, TRANSPORT
from check_step1_preview import FakePose, FakePoseV
from check_step2 import fixture_connection
from step1_pose_service import PoseServiceProcess
from step1_timeline import model_poses
from step2_feedback_service import FeedbackProcess


FEEDBACK_TRANSPORT = TRANSPORT + '''
import atexit
import threading
BaseNode = Node
class Node(BaseNode):
    def __init__(self):
        super().__init__()
        self.callbacks = {}
        self.lock = threading.Lock()
        atexit.register(self.finalize)
        threading.Thread(target=self.stream, daemon=True).start()
    def finalize(self):
        marker = os.environ.get('STEP2_FIXTURE_FINALIZER')
        if marker:
            with open(marker, 'w') as out:
                out.write('native callback owner finalized')
    def subscribe(self, message_type, topic, callback):
        if topic == '/world/factory/pose/info':
            raise RuntimeError('full factory stream overloads this feedback fixture')
        if os.environ.get('STEP2_FIXTURE_FAIL_SUBSCRIBE') and topic.endswith('/clock'):
            raise RuntimeError('fixture subscription refused')
        with self.lock:
            self.callbacks[topic] = callback
        return True
    def stream(self):
        tick = 0
        def pose(name, x, y, z, entity_id, stamp):
            return Obj(name=name, id=entity_id, header=Obj(stamp=stamp),
                       position=Obj(x=x, y=y, z=z), orientation=Obj(x=0., y=0., z=0., w=1.))
        while True:
            tick += 1
            stamp = Obj(sec=5+tick//100, nsec=(tick%100)*10000000)
            with self.lock:
                callbacks = list(self.callbacks.items())
            for topic, callback in callbacks:
                if topic.endswith('/clock'):
                    callback(Obj(sim=stamp))
                elif topic in ('/model/aisle_robot/pose', '/world/factory/model/aisle_robot/pose',
                               '/world/factory/dynamic_pose/info', '/world/factory/pose/info'):
                    poses = [pose('factory::aisle_robot', 0., -14., .005, 42, stamp),
                             pose('factory::aisle_robot_left_wheel', .25, -14., .115, 43, stamp),
                             pose('factory::aisle_robot_right_wheel', -.25, -14., .115, 44, stamp),
                             pose('factory::unrelated_box', 8., 8., 0., 90, stamp)]
                    callback(Obj(pose=poses, header=Obj(stamp=Obj(sec=0, nsec=0))))
                elif topic == '/model/step2_entrance_blue_beacon/pose':
                    # The fixed tile and wire share this static model. They
                    # are intentionally absent from dynamic_pose/info.
                    poses = [pose('step2_entrance_blue_beacon', -.45, -11., .020, 70, stamp)]
                    callback(Obj(pose=poses, header=Obj(stamp=stamp)))
            time.sleep(.01)
'''


class FeedbackProcessChecks(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='step2_feedback_check_')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        msgs = self.root/'gz/msgs10'
        msgs.mkdir(parents=True)
        for path in (msgs/'__init__.py', msgs.parent/'__init__.py'):
            path.write_text('')
        (msgs.parent/'transport13.py').write_text(FEEDBACK_TRANSPORT)
        (msgs/'fixture.py').write_text(MESSAGE)
        for module, name in (('pose_pb2', 'Pose'), ('pose_v_pb2', 'Pose_V'),
                             ('boolean_pb2', 'Boolean'), ('clock_pb2', 'Clock')):
            (msgs/(module+'.py')).write_text(f'from .fixture import Message\nclass {name}(Message): pass\n')
        environment = {'PYTHONPATH': str(self.root), 'GZ_PARTITION': 'step2_feedback_isolation_test',
                       'STEP1_FIXTURE_START_FAIL': '', 'STEP1_FIXTURE_NOISE': '',
                       'STEP2_FIXTURE_FAIL_SUBSCRIBE': '',
                       'STEP2_FIXTURE_FINALIZER': str(self.root/'unsafe-finalizer.txt')}
        self.environment = patch.dict(os.environ, environment)
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.connection = fixture_connection()

    def worker(self):
        worker = FeedbackProcess(self.connection.config, self.connection.tracked_models,
                                 self.connection.receive, self.connection.receive_clock,
                                 self.root/'feedback.log')
        self.addCleanup(worker.close)
        self.connection.feedback_process = worker
        return worker

    def wait_state(self):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            sample = self.connection.state()
            if sample and sample['pose_fresh']:
                return sample
            time.sleep(.01)
        self.fail('The real reader did not receive clock and pose data')

    def test_clock_robot_ids_beacon_and_wire_reach_parent_from_another_process(self):
        worker = self.worker()
        self.wait_state()
        self.assertNotEqual(worker.pid, os.getpid())
        self.assertEqual(self.connection.entity_ids['aisle_robot'], 42)
        self.assertEqual(self.connection.entity_ids['aisle_robot_left_wheel'], 43)
        self.assertTrue(self.connection.applied([('step2_entrance_blue_beacon', -.45, -11., .020)]))
        self.assertNotIn('unrelated_box', self.connection.model_samples)
        self.assertIn('partition=step2_feedback_isolation_test', (self.root/'feedback.log').read_text())

    def test_large_static_factory_stream_cannot_block_clock_or_robot_updates(self):
        worker = self.worker()
        self.assertNotIn('/world/factory/pose/info', worker.pose_topics)
        self.assertIn('/world/factory/dynamic_pose/info', worker.pose_topics)
        first = self.wait_state()['pose_time']
        time.sleep(.08)
        self.assertGreater(self.wait_state()['pose_time'], first)

    def test_feedback_continues_while_service_worker_holds_its_python_lock(self):
        feedback = self.worker()
        service = PoseServiceProcess(self.root/'service.log')
        self.addCleanup(service.close)
        before = self.wait_state()['pose_time']
        result, reply = service.request('/world/factory/set_pose/blocking', Probe(hold_gil_seconds=.3),
                                        Probe, object, 2000)
        after = self.wait_state()['pose_time']
        self.assertTrue(result and reply.data)
        self.assertGreater(after-before, .1)
        self.assertNotEqual(feedback.pid, service.pid)

    def test_repeated_worker_close_reaps_process_and_avoids_callback_finalizers(self):
        for _ in range(3):
            worker = self.worker()
            self.wait_state()
            process, thread = worker.process, worker.thread
            worker.close()
            worker.close()
            self.connection.feedback_process = None
            self.assertIsNotNone(process.poll())
            self.assertFalse(thread.is_alive())
            self.assertFalse((self.root/'unsafe-finalizer.txt').exists())

    def test_feedback_worker_failure_is_reported_instead_of_using_old_state(self):
        worker = self.worker()
        self.wait_state()
        worker.process.kill()
        worker.process.wait(timeout=2)
        with self.assertRaisesRegex(RuntimeError, 'Feedback worker'):
            self.connection.state()

    def test_partial_subscription_failure_exits_without_finalizing_native_callbacks(self):
        with patch.dict(os.environ, {'STEP2_FIXTURE_FAIL_SUBSCRIBE': '1'}):
            with self.assertRaisesRegex(RuntimeError, 'fixture subscription refused'):
                self.worker()
        self.assertFalse((self.root/'unsafe-finalizer.txt').exists())


class MovementClientChecks(unittest.TestCase):
    def connection(self):
        connection = fixture_connection()
        connection.Pose, connection.Pose_V, connection.Boolean = FakePose, FakePoseV, object
        connection.service = '/world/factory/set_pose_vector/blocking'
        connection.single_service = '/world/factory/set_pose/blocking'
        connection.request_failures = connection.request_retries = 0
        connection.last_failure = None
        connection.entity_ids = {'aisle_robot': 42, 'aisle_robot_left_wheel': 43,
                                 'aisle_robot_right_wheel': 44}
        return connection

    def target(self):
        return dict(x=0., y=-10.8, yaw=math.pi/2, left_spin=1., right_spin=1.)

    def feedback(self, connection, pose):
        connection.receive_clock(Obj(sim=Obj(sec=5, nsec=0)))
        pose.header = Obj(stamp=Obj(sec=5, nsec=0))
        connection.receive(Obj(pose=[pose]), connection.topics[2])

    def test_unapplied_batch_switches_to_single_robot_and_wheel_updates_with_entity_ids(self):
        connection, target, calls = self.connection(), self.target(), []
        initial = deepcopy(target)
        initial['y'] = -11.
        first = FakePose()
        Step2Connection.fill_pose(first, model_poses(initial, connection.name)[0])
        self.feedback(connection, first)
        def request(service, message, *args):
            calls.append((service, deepcopy(message)))
            if service == connection.single_service and message.name == connection.name:
                self.feedback(connection, message)
            return True, Obj(data=True)
        connection.requester.request = request
        connection.send(target)
        self.assertFalse(pose_matches(connection.state(), target))
        with redirect_stdout(io.StringIO()):
            connection.recover_motion(target, connection.state())
        self.assertTrue(pose_matches(connection.state(), target))
        self.assertEqual([service for service, _ in calls],
                         [connection.service]+[connection.single_service]*3)
        self.assertEqual([pose.id for _, pose in calls[1:]], [42, 43, 44])
        self.assertEqual({pose.name for _, pose in calls[1:]}, set(connection.entity_ids))
        self.assertEqual(connection.motion_recoveries, 1)
        self.assertFalse(connection.use_vector)
        self.assertEqual(connection.last_motion_mismatch['observed']['y'], -11.)

    def test_accepted_single_pose_without_actual_movement_does_not_pass_confirmation(self):
        connection, target = self.connection(), self.target()
        initial = deepcopy(target)
        initial['y'] = -11.
        first = FakePose()
        Step2Connection.fill_pose(first, model_poses(initial, connection.name)[0])
        self.feedback(connection, first)
        connection.requester.request = lambda *args: (True, Obj(data=True))
        with redirect_stdout(io.StringIO()):
            connection.recover_motion(target, connection.state())
        self.assertFalse(pose_matches(connection.state(), target))

    def test_reported_27mm_lag_is_never_accepted_as_a_completed_movement(self):
        connection, target = self.connection(), self.target()
        sample = {**target, 'y': target['y']-.027, 'z': .005, 'pose_time': 5.1}
        connection.command_pose_time = 5.04
        self.assertFalse(connection.motion_confirmed(sample, target))

    def test_matching_position_from_before_the_command_does_not_advance_route(self):
        connection, target = self.connection(), self.target()
        sample = {**target, 'z': .005, 'pose_time': 5.0}
        connection.command_pose_time = 5.04
        self.assertFalse(connection.motion_confirmed(sample, target))
        sample['pose_time'] = 5.04
        self.assertTrue(connection.motion_confirmed(sample, target))

    def test_millimetre_step_waits_for_new_pose_even_inside_spatial_tolerance(self):
        connection, target = self.connection(), self.target()
        sample = {**target, 'y': target['y']-.001, 'z': .005, 'pose_time': 5.0}
        connection.command_pose_time = 5.04
        self.assertTrue(pose_matches(sample, target))
        self.assertFalse(connection.motion_confirmed(sample, target))
        sample['pose_time'] = 5.04
        self.assertTrue(connection.motion_confirmed(sample, target))

    def test_new_receipt_without_a_simulation_pose_stamp_cannot_confirm_motion(self):
        connection, target = self.connection(), self.target()
        sample = {**target, 'z': .005, 'pose_time': None, 'received': time.monotonic()}
        connection.command_pose_time = 5.04
        self.assertFalse(connection.motion_confirmed(sample, target))

    def test_controller_constructs_no_native_subscription_node_and_closes_both_workers(self):
        class NoNode:
            def __init__(self):
                raise AssertionError('Controller must not own a native subscription node')
        class Worker:
            pid = 123
            closed = False
            def __init__(self, *args):
                pass
            def close(self):
                self.closed = True
        config = fixture_connection().config
        with patch('animate_step2.creation_message_type', return_value=object), \
                patch('animate_step2.transport_imports', return_value=(NoNode, FakePose, FakePoseV, object, object)), \
                patch('animate_step2.PoseServiceProcess', Worker), \
                patch('animate_step2.FeedbackProcess', Worker):
            connection = Step2Connection(config, [])
        self.assertTrue(connection.diagnostics()['feedback_subscriptions_isolated'])
        connection.close()
        self.assertTrue(connection.requester.closed)
        self.assertTrue(connection.feedback_process.closed)


if __name__ == '__main__':
    unittest.main(verbosity=1)

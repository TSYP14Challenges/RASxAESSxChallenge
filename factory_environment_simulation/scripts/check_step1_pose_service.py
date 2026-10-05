#!/usr/bin/env python3
"""Exercise the real pose subprocess with a process-local transport fixture."""
import json
import os
from pathlib import Path
import runpy
import tempfile
import threading
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

from step1_pose_service import PoseServiceProcess


class Probe:
    DESCRIPTOR = Obj(full_name='gz.msgs.Pose')

    def __init__(self, **payload):
        self.payload = payload

    def SerializeToString(self):
        return json.dumps(self.payload).encode('utf-8')


class VectorProbe(Probe):
    DESCRIPTOR = Obj(full_name='gz.msgs.Pose_V')


class FactoryProbe(Probe):
    DESCRIPTOR = Obj(full_name='gz.msgs.EntityFactory')


TRANSPORT = '''import ctypes
import json
import os
import time
from types import SimpleNamespace as Obj
subscribed = False
calls = 0
class Node:
    def __init__(self):
        if os.environ.get('STEP1_FIXTURE_START_FAIL'):
            raise RuntimeError('fixture transport cannot start')
        if os.environ.get('STEP1_FIXTURE_NOISE'):
            print('native transport startup diagnostic', flush=True)
    def subscribe(self, *args):
        global subscribed
        subscribed = True
        return True
    def request(self, service, message, request_type, response_type, timeout):
        global calls
        calls += 1
        # Reproduce first-call success followed by timeouts in a subscribed
        # process. All Nodes in this interpreter share the same flag.
        if subscribed and calls > 1:
            return False, None
        data = message.payload
        if data.get('exit'):
            os._exit(24)
        if data.get('hang'):
            time.sleep(30)
        if data.get('error'):
            raise RuntimeError('fixture service exception')
        if data.get('hold_gil_seconds'):
            ctypes.PyDLL(None).usleep(int(data['hold_gil_seconds']*1000000))
        if data.get('record'):
            with open(data['record'], 'a') as stream:
                stream.write(json.dumps({'pid': os.getpid(),
                    'partition': os.environ.get('GZ_PARTITION'),
                    'protobuf_implementation': os.environ.get('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'),
                    'subscribed': subscribed, 'service': service,
                    'timeout': timeout, 'payload': data,
                    'type': request_type.__name__})+'\\n')
        if data.get('timeout'):
            return False, None
        return True, Obj(data=not data.get('reject', False))
'''

MESSAGE = '''import json
class Message:
    def ParseFromString(self, data):
        self.payload = json.loads(data)
'''


class ProcessChecks(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='step1_pose_process_check_')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        gz = self.root/'gz'
        msgs = gz/'msgs10'
        msgs.mkdir(parents=True)
        for path in (gz/'__init__.py', msgs/'__init__.py'):
            path.write_text('')
        (gz/'transport13.py').write_text(TRANSPORT)
        (msgs/'fixture.py').write_text(MESSAGE)
        (msgs/'pose_pb2.py').write_text('from .fixture import Message\nclass Pose(Message): pass\n')
        (msgs/'pose_v_pb2.py').write_text('from .fixture import Message\nclass Pose_V(Message): pass\n')
        (msgs/'entity_factory_pb2.py').write_text('from .fixture import Message\nclass EntityFactory(Message): pass\n')
        (msgs/'boolean_pb2.py').write_text('class Boolean: pass\n')
        environment = {'PYTHONPATH': str(self.root), 'GZ_PARTITION': 'step1_isolation_test',
                       'STEP1_FIXTURE_START_FAIL': '', 'STEP1_FIXTURE_NOISE': ''}
        self.environment = patch.dict(os.environ, environment)
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def worker(self):
        worker = PoseServiceProcess(self.root/'worker.log')
        self.addCleanup(worker.close)
        return worker

    def send(self, worker, message=None, timeout=2000):
        message = message or Probe()
        service = '/world/factory/'+('set_pose_vector' if isinstance(message, VectorProbe) else 'set_pose')
        return worker.request(service, message, type(message), object, timeout)

    def test_persistent_worker_isolates_requests_from_process_wide_subscriptions(self):
        legacy = runpy.run_path(str(self.root/'gz/transport13.py'))['Node']()
        legacy.subscribe(object, '/world/factory/clock', lambda message: None)
        self.assertTrue(legacy.request('/world/factory/set_pose', Probe(), Probe, object, 2000)[0])
        self.assertFalse(legacy.request('/world/factory/set_pose', Probe(), Probe, object, 2000)[0])
        worker = self.worker()
        record = self.root/'requests.jsonl'
        for index in range(100):
            message = (VectorProbe if index % 2 else Probe)(record=str(record), index=index)
            result, response = self.send(worker, message)
            self.assertTrue(result)
            self.assertTrue(response.data)
        rows = [json.loads(line) for line in record.read_text().splitlines()]
        self.assertEqual(len(rows), 100)
        self.assertEqual({row['pid'] for row in rows}, {worker.pid})
        self.assertNotEqual(worker.pid, os.getpid())
        self.assertEqual({row['partition'] for row in rows}, {'step1_isolation_test'})
        self.assertTrue(all(not row['subscribed'] for row in rows))
        self.assertEqual([row['payload']['index'] for row in rows], list(range(100)))
        self.assertEqual({row['type'] for row in rows}, {'Pose', 'Pose_V'})

    def test_parent_feedback_can_run_while_worker_holds_the_python_lock(self):
        worker = self.worker()
        stop = threading.Event()
        ticks = []
        def feedback():
            while not stop.wait(.01):
                ticks.append(time.monotonic())
        thread = threading.Thread(target=feedback)
        thread.start()
        try:
            start = time.monotonic()
            self.assertTrue(self.send(worker, Probe(hold_gil_seconds=.25))[0])
            end = time.monotonic()
        finally:
            stop.set()
            thread.join(timeout=1)
        self.assertGreaterEqual(len([tick for tick in ticks if start < tick < end]), 5)

    def test_model_creation_uses_the_same_isolated_worker_and_preserves_sdf(self):
        worker = self.worker()
        record = self.root/'creations.jsonl'
        sdf = '<sdf version="1.9"><model name="blue_gateway"><pose>-0.45 -11 0.016 0 0 0</pose></model></sdf>'
        for index in range(3):
            self.assertTrue(self.send(worker)[0])
            message = FactoryProbe(record=str(record), name='blue_gateway_'+str(index),
                                   sdf=sdf, allow_renaming=False)
            result, response = worker.request('/world/factory/create', message, FactoryProbe, object, 5000)
            self.assertTrue(result and response.data)
        rows = [json.loads(line) for line in record.read_text().splitlines()]
        self.assertEqual(len(rows), 3)
        self.assertEqual({row['type'] for row in rows}, {'EntityFactory'})
        self.assertEqual({row['service'] for row in rows}, {'/world/factory/create'})
        self.assertEqual({row['pid'] for row in rows}, {worker.pid})
        self.assertNotEqual(worker.pid, os.getpid())
        self.assertTrue(all(not row['subscribed'] for row in rows))
        self.assertTrue(all(row['payload']['sdf'] == sdf for row in rows))
        self.assertTrue(all(row['payload']['allow_renaming'] is False for row in rows))

    def test_step2_parser_selection_is_inherited_by_the_isolated_worker(self):
        record = self.root/'parser.jsonl'
        with patch.dict(os.environ, {'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION': 'python'}):
            worker = self.worker()
        self.assertTrue(self.send(worker, Probe(record=str(record)))[0])
        rows = [json.loads(line) for line in record.read_text().splitlines()]
        self.assertEqual(rows[0]['protobuf_implementation'], 'python')
        self.assertNotEqual(rows[0]['pid'], os.getpid())

    def test_false_and_missing_replies_are_preserved(self):
        worker = self.worker()
        result, response = self.send(worker, Probe(reject=True))
        self.assertTrue(result)
        self.assertFalse(response.data)
        result, response = self.send(worker, Probe(timeout=True))
        self.assertFalse(result)
        self.assertIsNone(response)

    def test_service_exception_does_not_destroy_the_worker(self):
        worker = self.worker()
        with self.assertRaisesRegex(RuntimeError, 'fixture service exception'):
            self.send(worker, Probe(error=True))
        self.assertTrue(self.send(worker)[0])

    def test_unexpected_worker_exit_is_reported_and_cleaned_up(self):
        worker = self.worker()
        process = worker.process
        with self.assertRaisesRegex(RuntimeError, 'worker exited'):
            self.send(worker, Probe(exit=True))
        self.assertIsNotNone(process.poll())
        self.assertIsNone(worker.process)

    def test_hung_worker_is_stopped_without_reusing_an_inflight_request(self):
        worker = self.worker()
        process = worker.process
        start = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'worker stopped replying'):
            self.send(worker, Probe(hang=True), timeout=100)
        self.assertLess(time.monotonic()-start, 6)
        self.assertIsNotNone(process.poll())
        self.assertIsNone(worker.process)

    def test_worker_startup_failure_has_a_clear_message(self):
        with patch.dict(os.environ, {'STEP1_FIXTURE_START_FAIL': '1'}):
            with self.assertRaisesRegex(RuntimeError, 'fixture transport cannot start'):
                self.worker()

    def test_native_diagnostic_output_does_not_corrupt_the_reply_protocol(self):
        with patch.dict(os.environ, {'STEP1_FIXTURE_NOISE': '1'}):
            worker = self.worker()
        self.assertTrue(self.send(worker)[0])
        self.assertIn('startup diagnostic', (self.root/'worker.log').read_text())


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ProcessChecks)
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    destination = Path(__file__).resolve().parents[1]/'step1/pose_service_validation.json'
    destination.write_text(json.dumps({
        'passed': True, 'tests': result.testsRun, 'real_subprocesses_tested': True,
        'native_gazebo_transport_tested_here': False,
        'note': 'Real worker process and pipe protocol checked with a process-local transport fixture.'
    }, indent=2)+'\n')

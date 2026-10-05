#!/usr/bin/env python3
"""Keep blocking Gazebo service requests in a process with no subscriptions.

Gazebo Transport nodes share process-wide resources. A second Node or a Python
thread does not isolate blocking requests from subscription callbacks. This
persistent subprocess is started with exec, so its transport state and Python
interpreter are independent. Only one pose or model-creation request is in flight.
"""
import base64
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
from types import SimpleNamespace


class PoseServiceProcess:
    def __init__(self, log_path):
        self.process = None
        self.buffer = b''
        self.sequence = 0
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open('wb', buffering=0)
        try:
            # Popen inherits GZ_PARTITION and starts a fresh interpreter. Avoid
            # multiprocessing's fork mode: it inherits live transport locks.
            self.process = subprocess.Popen(
                [sys.executable, '-B', '-u', str(Path(__file__).resolve()), '--worker'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
            self.pid = self.process.pid
            hello = self.read_reply(10.0)
            if not hello.get('ready'):
                raise RuntimeError('Pose worker could not start: '+hello.get('error', 'invalid startup reply'))
            if hello.get('pid') != self.pid:
                raise RuntimeError('Pose worker returned an unexpected process identity')
        except BaseException:
            self.close()
            raise

    def read_reply(self, seconds):
        deadline = time.monotonic()+seconds
        while True:
            if b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                try:
                    message = json.loads(line)
                except (ValueError, UnicodeError):
                    # Native Gazebo diagnostics may also use stdout.
                    self.log.write(line+b'\n')
                    continue
                if isinstance(message, dict):
                    return message
                self.log.write(line+b'\n')
                continue
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise RuntimeError('Pose worker stopped replying; inspect '+str(self.log_path))
            readable, _, _ = select.select([self.process.stdout], [], [], min(.5, remaining))
            if not readable:
                continue
            chunk = os.read(self.process.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError('Pose worker exited; inspect '+str(self.log_path))
            self.buffer += chunk
            if len(self.buffer) > 65536:
                raise RuntimeError('Pose worker output is invalid; inspect '+str(self.log_path))

    def request(self, service, message, request_type, response_type, timeout):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('Pose worker is not running; inspect '+str(self.log_path))
        self.sequence += 1
        request = {'id': self.sequence, 'service': service,
                   'request_type': request_type.DESCRIPTOR.full_name,
                   'payload': base64.b64encode(message.SerializeToString()).decode('ascii'),
                   'timeout_ms': timeout}
        try:
            self.process.stdin.write((json.dumps(request)+'\n').encode('utf-8'))
            reply = self.read_reply(timeout/1000+3.0)
            if reply.get('id') != self.sequence:
                raise RuntimeError('Pose worker reply is out of order')
        except (OSError, RuntimeError):
            # A worker with an unknown in-flight request must never be reused.
            self.close()
            raise
        if reply.get('error'):
            raise RuntimeError(reply['error'])
        data = reply.get('data')
        response = None if data is None else SimpleNamespace(data=bool(data))
        return bool(reply.get('result')), response

    def close(self):
        process = self.process
        if process is not None:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
            if process.stdout is not None:
                process.stdout.close()
            self.process = None
        if not self.log.closed:
            self.log.close()

    def scene_models(self, service, names, timeout=2000):
        """Read selected scene identities once, without a full-scene subscription."""
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('Service worker is not running')
        self.sequence += 1
        request = {'id': self.sequence, 'operation': 'scene_models', 'service': service,
                   'names': list(names), 'timeout_ms': timeout}
        try:
            self.process.stdin.write((json.dumps(request)+'\n').encode('utf-8'))
            reply = self.read_reply(timeout/1000+3.)
            if reply.get('id') != self.sequence:
                raise RuntimeError('Scene query reply is out of order')
        except (OSError, RuntimeError):
            self.close()
            raise
        if reply.get('error') or not reply.get('result'):
            raise RuntimeError('Could not read the Gazebo scene: '+str(reply.get('error', 'timeout')))
        return reply['models']


def emit(message):
    print(json.dumps(message), flush=True)


def worker_main():
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.boolean_pb2 import Boolean
        node = Node()
    except Exception as error:
        emit({'ready': False, 'error': str(error)})
        return 1
    types = {'gz.msgs.Pose': Pose, 'gz.msgs.Pose_V': Pose_V}
    print(f'Pose worker ready: pid={os.getpid()}, '
          f'partition={os.environ.get("GZ_PARTITION", "default")}, no subscriptions.',
          file=sys.stderr, flush=True)
    emit({'ready': True, 'pid': os.getpid()})
    # No subscribe() calls in this interpreter: keep the service path free of
    # Python clock/pose callbacks and their transport/GIL locking interaction.
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            if request.get('operation') == 'scene_models':
                from gz.msgs10.empty_pb2 import Empty
                from gz.msgs10.scene_pb2 import Scene
                result, scene = node.request(request['service'], Empty(), Empty, Scene,
                                             int(request['timeout_ms']))
                models = {}
                if result and scene is not None:
                    wanted = set(request['names'])
                    for model in scene.model:
                        name = model.name.split('::')[-1]
                        if name not in wanted:
                            continue
                        visuals = {}
                        for link in model.link:
                            for visual in link.visual:
                                key = link.name.split('::')[-1]+'::'+visual.name.split('::')[-1]
                                color = visual.material.diffuse
                                visuals[key] = {'id': int(visual.id), 'parent_id': int(link.id),
                                                'diffuse': [color.r, color.g, color.b, color.a]}
                        models[name] = {'id': int(model.id), 'visuals': visuals}
                emit({'id': request['id'], 'result': bool(result and scene is not None), 'models': models})
                continue
            if request['request_type'] == 'gz.msgs.EntityFactory' and request['request_type'] not in types:
                from gz.msgs10.entity_factory_pb2 import EntityFactory
                types['gz.msgs.EntityFactory'] = EntityFactory
            elif request['request_type'] == 'gz.msgs.Entity' and request['request_type'] not in types:
                from gz.msgs10.entity_pb2 import Entity
                types['gz.msgs.Entity'] = Entity
            elif request['request_type'] == 'gz.msgs.Visual' and request['request_type'] not in types:
                from gz.msgs10.visual_pb2 import Visual
                types['gz.msgs.Visual'] = Visual
            message_type = types[request['request_type']]
            message = message_type()
            message.ParseFromString(base64.b64decode(request['payload'], validate=True))
            timeout = int(request['timeout_ms'])
            if not 1 <= timeout <= 10000:
                raise ValueError('Invalid pose request timeout')
            result, response = node.request(request['service'], message, message_type, Boolean, timeout)
            data = None if response is None else bool(response.data)
            if not result or not data:
                print(json.dumps({'pose_request_failure': request['id'], 'service': request['service'],
                                  'request_type': request['request_type'], 'timeout_ms': timeout,
                                  'result': bool(result), 'data': data}), file=sys.stderr, flush=True)
            emit({'id': request['id'], 'result': bool(result), 'data': data})
        except Exception as error:
            print('Pose worker error: '+str(error), file=sys.stderr, flush=True)
            emit({'id': request.get('id'), 'result': False, 'data': None, 'error': str(error)})
    return 0


if __name__ == '__main__':
    if sys.argv[1:] != ['--worker']:
        raise SystemExit('This helper is started automatically by animate_step1.py.')
    raise SystemExit(worker_main())

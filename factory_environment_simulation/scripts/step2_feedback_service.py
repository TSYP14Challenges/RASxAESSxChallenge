#!/usr/bin/env python3
"""Own Gazebo subscriptions in an exec-started process, separate from control.

The controller receives filtered JSON data and never owns a native callback.
On close the worker is terminated and reaped before controller shutdown. This
avoids destroying Transport's Python callback references during finalization.
"""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
from types import SimpleNamespace as Obj


def pose_topics(config, names):
    robot, world = config['model_name'], config['world_name']
    # The full-scene pose/info packet includes every shelf, link and visual.
    # With the Python protobuf parser that packet alone can saturate the
    # callback thread. Use only the periodic robot / active-model streams.
    topics = [f'/model/{robot}/pose', f'/world/{world}/model/{robot}/pose',
              f'/world/{world}/dynamic_pose/info']
    for name in sorted(set(names)-{robot, robot+'_left_wheel', robot+'_right_wheel'}):
        topics.extend((f'/model/{name}/pose', f'/world/{world}/model/{name}/pose'))
    return topics


class FeedbackProcess:
    def __init__(self, config, model_names, receive, receive_clock, log_path):
        self.process = self.thread = None
        self.error = None
        self.pose_topics = []
        self.stopping = threading.Event()
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open('wb', buffering=0)
        self.receive, self.receive_clock = receive, receive_clock
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-B', '-u', str(Path(__file__).resolve()), '--worker'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
            self.pid = self.process.pid
            self.process.stdin.write((json.dumps({'config': config, 'models': sorted(model_names)})+'\n')
                                     .encode('utf-8'))
            self.thread = threading.Thread(target=self.read_loop, name='step2-feedback', daemon=True)
            self.ready = threading.Event()
            self.thread.start()
            if not self.ready.wait(10):
                raise RuntimeError('Feedback worker did not start; inspect '+str(self.log_path))
            self.check()
        except BaseException:
            self.close()
            raise

    def check(self):
        if self.error:
            raise RuntimeError(self.error+'; inspect '+str(self.log_path))
        if not self.stopping.is_set() and self.process.poll() is not None:
            raise RuntimeError('Feedback worker exited; inspect '+str(self.log_path))

    def dispatch(self, data):
        if 'ready' in data:
            if not data['ready'] or data.get('pid') != self.pid:
                self.error = 'Feedback worker could not start: '+data.get('error', 'invalid identity')
            self.pose_topics = data.get('pose_topics', [])
            self.ready.set()
        elif data.get('kind') == 'clock':
            sec, nsec = data['sim']
            self.receive_clock(Obj(sim=Obj(sec=sec, nsec=nsec)))
        elif data.get('kind') == 'pose':
            poses = []
            for p in data['poses']:
                sec, nsec = p['stamp']
                poses.append(Obj(name=p['name'], id=p['id'], header=Obj(stamp=Obj(sec=sec, nsec=nsec)),
                                 position=Obj(**p['position']), orientation=Obj(**p['orientation'])))
            self.receive(Obj(pose=poses, header=Obj(stamp=Obj(sec=0, nsec=0))), data['source'])

    def read_loop(self):
        buffer = b''
        try:
            while not self.stopping.is_set():
                readable, _, _ = select.select([self.process.stdout], [], [], .2)
                if not readable:
                    continue
                chunk = os.read(self.process.stdout.fileno(), 65536)
                if not chunk:
                    if not self.stopping.is_set() and not self.error:
                        self.error = 'Feedback worker closed its output'
                    break
                buffer += chunk
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        data = json.loads(line)
                    except (ValueError, UnicodeError):
                        self.log.write(line+b'\n')
                        continue
                    if isinstance(data, dict):
                        self.dispatch(data)
                if len(buffer) > 1024*1024:
                    raise ValueError('Feedback worker output exceeds the protocol limit')
        except Exception as error:
            if not self.stopping.is_set() and not self.error:
                self.error = 'Feedback reader failed: '+str(error)
        finally:
            self.ready.set()

    def close(self):
        self.stopping.set()
        if self.process is not None:
            # SIGTERM ends the native callback owner without invoking Python
            # finalizers. This worker has no mission files or services to flush.
            if self.process.poll() is None:
                self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
            if self.thread is not None:
                self.thread.join(timeout=2)
            self.process.stdin.close()
            self.process.stdout.close()
            self.process = None
        if not self.log.closed:
            self.log.close()


def worker_main():
    # This helper is also safe when launched directly by a subprocess fixture.
    os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
    output_lock = threading.Lock()
    def emit(data):
        with output_lock:
            print(json.dumps(data, separators=(',', ':')), flush=True)
    try:
        setup = json.loads(sys.stdin.readline())
        config, names = setup['config'], set(setup['models'])
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.clock_pb2 import Clock
        node = Node()
        world = config['world_name']
        topics = pose_topics(config, names)
        callbacks = []
        for topic in topics:
            def callback(message, source=topic):
                poses = []
                for pose in message.pose:
                    if pose.name.split('::')[-1] not in names:
                        continue
                    stamp = pose.header.stamp
                    if not stamp.sec and not stamp.nsec:
                        stamp = message.header.stamp
                    poses.append({'name': pose.name, 'id': int(pose.id),
                                  'stamp': [stamp.sec, stamp.nsec],
                                  'position': {k: getattr(pose.position, k) for k in ('x', 'y', 'z')},
                                  'orientation': {k: getattr(pose.orientation, k) for k in ('x', 'y', 'z', 'w')}})
                if poses:
                    emit({'kind': 'pose', 'source': source, 'poses': poses})
            callbacks.append(callback)
            if not node.subscribe(Pose_V, topic, callback):
                raise RuntimeError('Could not subscribe to '+topic)
        def receive_clock(message):
            emit({'kind': 'clock', 'sim': [message.sim.sec, message.sim.nsec]})
        if not node.subscribe(Clock, f'/world/{world}/clock', receive_clock):
            raise RuntimeError('Could not subscribe to the simulation clock')
        print(f'Feedback worker ready: pid={os.getpid()}, partition={os.environ.get("GZ_PARTITION", "default")}; '
              'small model streams only; full-scene pose/info is excluded.', file=sys.stderr, flush=True)
        emit({'ready': True, 'pid': os.getpid(), 'pose_topics': topics})
        # The parent terminates this process on every exit path. Keeping all
        # callback owners alive avoids unsafe native teardown on normal exit.
        threading.Event().wait()
    except BaseException as error:
        emit({'ready': False, 'pid': os.getpid(), 'error': str(error)})
        # Even partial subscription startup can own native Python callbacks.
        os._exit(1)


if __name__ == '__main__':
    if sys.argv[1:] != ['--worker']:
        raise SystemExit('This helper is started automatically by animate_step2.py.')
    worker_main()

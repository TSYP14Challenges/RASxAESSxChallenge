#!/usr/bin/env python3
"""Keep simulation time separate from robot pose messages; no Gazebo imports."""
import math
import threading
import time


def pose_stamp(message):
    """An unset protobuf stamp reads as zero; it is not a running clock."""
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    value = stamp.sec + stamp.nsec * 1e-9
    return value if value > 0 else None


class Feedback:
    def __init__(self, config):
        self.name = config['model_name']
        self.max_pose_age = config.get('pose_max_age', .25)
        self.prefer_newest_pose = config.get('prefer_newest_pose', False)
        world = config['world_name']
        self.clock_topic = f'/world/{world}/clock'
        # Prefer the periodic robot publisher over the scene's initial snapshot.
        self.topics = [f'/model/{self.name}/pose',
                       f'/world/{world}/model/{self.name}/pose',
                       f'/world/{world}/dynamic_pose/info',
                       f'/world/{world}/pose/info']
        self.lock = threading.Lock()
        self.samples, self.clock, self.time_reset = {}, None, False

    def receive_clock(self, message):
        sim = message.sim.sec + message.sim.nsec * 1e-9
        with self.lock:
            if self.clock is not None and sim < self.clock['sim_time'] - .1:
                self.time_reset = True
            self.clock = {'sim_time': sim, 'clock_received': time.monotonic()}

    def receive(self, message, source):
        for pose in message.pose:
            if pose.name != self.name and not pose.name.endswith('::' + self.name):
                continue
            p, q = pose.position, pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, yaw)):
                continue
            # PosePublisher may stamp each Pose instead of the Pose_V envelope.
            stamp = pose_stamp(pose)
            if stamp is None:
                stamp = pose_stamp(message)
            with self.lock:
                old = self.samples.get(source)
                if old and stamp is not None and old['pose_time'] is not None \
                        and stamp < old['pose_time']:
                    return
                self.samples[source] = {
                    'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw,
                    'entity_id': int(getattr(pose, 'id', 0)),
                    'pose_time': stamp, 'received': time.monotonic(), 'topic': source,
                    'clock_at_pose': self.clock['sim_time'] if self.clock else None}
            return

    def state(self):
        with self.lock:
            if self.clock is None or not self.samples:
                return None
            now, sim = time.monotonic(), self.clock['sim_time']
            ordered = [self.samples[t] for t in self.topics if t in self.samples]
            def fresh(sample):
                stamp = sample['pose_time']
                if stamp is None:
                    stamp = sample['clock_at_pose']
                return now - sample['received'] <= 1.0 and \
                    (stamp is None or sim - stamp <= self.max_pose_age)
            recent = [sample for sample in ordered if fresh(sample)]
            if recent and self.prefer_newest_pose:
                # A recently received model stream may still describe an older
                # simulation step than the world stream. Never let it mask a
                # newer stamped pose. Unstamped scene snapshots cannot override
                # a valid stamped stream merely by arriving later.
                stamped = [s for s in recent if s['pose_time'] is not None]
                sample = max(stamped, key=lambda s: s['pose_time']) if stamped else recent[0]
            else:
                sample = recent[0] if recent else max(ordered, key=lambda s: s['received'])
            return {**sample, **self.clock, 'pose_fresh': bool(recent),
                    'time_reset': self.time_reset}

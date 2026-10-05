#!/usr/bin/env python3
"""Check Step 2 message bootstrap and real protobuf decoding without a simulator.

--native checks the installed Gazebo message classes and Transport extension.
Offline tests use real protobuf classes with a small, private schema fixture;
they are distinct from native Gazebo transport and rendering tests.
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

# animate_step2 selects the parser before we load protobuf in this process.
from animate_step2 import creation_message_type, validate_message_types
from check_step2 import fixture_connection
from step2_events import PACKAGE, load_config


def protobuf_types():
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    file = descriptor_pb2.FileDescriptorProto(name='step2_message_fixture.proto', package='gz.msgs')
    fields = {
        'Time': [('sec', 'int64'), ('nsec', 'int32')],
        'Header': [('stamp', 'Time')],
        'Vector3d': [('x', 'double'), ('y', 'double'), ('z', 'double')],
        'Quaternion': [('x', 'double'), ('y', 'double'), ('z', 'double'), ('w', 'double')],
        'Pose': [('name', 'string'), ('position', 'Vector3d'), ('orientation', 'Quaternion'), ('header', 'Header')],
        'Pose_V': [('pose', 'Pose', 'repeated'), ('header', 'Header')],
        'Clock': [('sim', 'Time')],
        'Boolean': [('data', 'bool')],
        'EntityFactory': [('name', 'string'), ('sdf', 'string'), ('allow_renaming', 'bool'), ('pose', 'Pose')],
    }
    primitive = {'double': 1, 'int64': 3, 'int32': 5, 'bool': 8, 'string': 9}
    for name, definitions in fields.items():
        message = file.message_type.add(name=name)
        for number, definition in enumerate(definitions, 1):
            field_name, field_type, *repeated = definition
            field = message.field.add(name=field_name, number=number,
                                      label=3 if repeated else 1, type=primitive.get(field_type, 11))
            if field_type not in primitive:
                field.type_name = '.gz.msgs.'+field_type
    pool = descriptor_pool.DescriptorPool()
    pool.Add(file)
    factory = message_factory.MessageFactory(pool)
    def get(name):
        descriptor = pool.FindMessageTypeByName('gz.msgs.'+name)
        if hasattr(message_factory, 'GetMessageClass'):
            return message_factory.GetMessageClass(descriptor)
        return factory.GetPrototype(descriptor)
    return SimpleNamespace(**{name: get(name) for name in fields})


def protobuf_available():
    try:
        return importlib.util.find_spec('google.protobuf') is not None
    except ModuleNotFoundError:
        return False


class BootstrapChecks(unittest.TestCase):
    def test_fresh_controller_overrides_cpp_parser_before_message_imports(self):
        environment = {**os.environ, 'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION': 'cpp'}
        code = ('import os, sys; import animate_step2; '
                'assert os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] == "python"; '
                'assert "gz.msgs10.pose_pb2" not in sys.modules; '
                'assert "google.protobuf" not in sys.modules')
        result = subprocess.run([sys.executable, '-B', '-c', code], cwd=PACKAGE/'scripts',
                                env=environment, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_unregistered_nested_pose_is_reported_during_preflight(self):
        class UnregisteredVector:
            def __init__(self):
                self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nsec=0))
            @property
            def pose(self):
                raise TypeError("No message class registered for 'gz.msgs.Pose'")
        with self.assertRaisesRegex(TypeError, "No message class registered"):
            validate_message_types(object, UnregisteredVector, object, object, object)


@unittest.skipUnless(protobuf_available(), 'Real protobuf tests need the existing python3-protobuf package.')
class ProtobufChecks(unittest.TestCase):
    def test_real_nested_messages_round_trip_after_factory_is_loaded(self):
        types = protobuf_types()
        self.assertTrue(validate_message_types(types.Pose, types.Pose_V, types.Boolean,
                                               types.Clock, types.EntityFactory))
        from google.protobuf.internal import api_implementation
        self.assertEqual(api_implementation.Type(), 'python')

    def test_transport_is_initialized_before_entity_factory(self):
        types, calls = protobuf_types(), []
        class FactoryModule:
            def __getattr__(self, name):
                if name != 'EntityFactory':
                    raise AttributeError(name)
                calls.append('factory')
                return types.EntityFactory
        def transport():
            calls.append('transport')
            return object, types.Pose, types.Pose_V, types.Boolean, types.Clock
        with patch('animate_step2.transport_imports', transport), \
                patch.dict(sys.modules, {'gz.msgs10.entity_factory_pb2': FactoryModule()}):
            self.assertIs(creation_message_type(), types.EntityFactory)
        self.assertEqual(calls[0], 'transport')
        self.assertIn('factory', calls[1:])

    def test_real_serialized_feedback_reaches_robot_gateway_and_fire_callbacks(self):
        types, config = protobuf_types(), load_config()
        connection = fixture_connection(config=config)
        vector = types.Pose_V()
        expected = [(config['model_name'], 0., -14., .005),
                    ('step2_entrance_blue_beacon', *config['entrance_beacon_position']),
                    ('step2_fire_01_beacon', -15.0, -7.0, .016)]
        for name, x, y, z in expected:
            pose = vector.pose.add()
            pose.name = name
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.
            pose.header.stamp.sec = 1
        wire, failures = vector.SerializeToString(), []
        clock = types.Clock()
        clock.sim.sec = 1
        connection.receive_clock(clock)
        def callback():
            try:
                incoming = types.Pose_V()
                incoming.ParseFromString(wire)
                connection.receive(incoming, '/world/factory/dynamic_pose/info')
            except Exception as error:
                failures.append(error)
        thread = threading.Thread(target=callback)
        thread.start()
        thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(failures, [])
        self.assertTrue(connection.state()['pose_fresh'])
        self.assertEqual((connection.state()['x'], connection.state()['y']), (0., -14.))
        self.assertTrue(connection.applied(expected[1:]))

    def test_positive_preflight_cannot_hide_an_unregistered_pose(self):
        types = protobuf_types()
        class BrokenVector:
            def __init__(self):
                self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nsec=0))
            @property
            def pose(self):
                raise TypeError("No message class registered for 'gz.msgs.Pose'")
        with patch('animate_step2.transport_imports', return_value=(object, types.Pose,
                        BrokenVector, types.Boolean, types.Clock)), \
                patch.dict(sys.modules, {'gz.msgs10.entity_factory_pb2': SimpleNamespace(EntityFactory=types.EntityFactory)}):
            with self.assertRaisesRegex(RuntimeError, 'check failed before starting the mission'):
                creation_message_type()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', action='store_true')
    args = parser.parse_args()
    if args.native:
        creation_message_type()
        print('PASS: installed Gazebo messages decode nested Pose, positions, orientations, clock and beacon requests (Python parser).')
        return
    suite = unittest.TestSuite()
    for cls in (BootstrapChecks, ProtobufChecks):
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(cls))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    (PACKAGE/'step2/message_validation.json').write_text(json.dumps({
        'passed': True, 'tests': result.testsRun, 'skipped': len(result.skipped),
        'real_protobuf_round_trips_tested': protobuf_available(),
        'native_gazebo_transport_tested_here': False,
        'protobuf_implementation': 'python',
        'note': 'Bootstrap, actual protobuf decoding and feedback callback checks use private schema fixtures.'
    }, indent=2)+'\n')


if __name__ == '__main__':
    main()

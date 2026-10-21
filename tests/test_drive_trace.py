"""Offline regression tests against pinned upstream code; no ROS/hardware needed."""
import copy
import hashlib
import importlib.util
import json
import os
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(ROOT, 'harada-tsubakino', 'src')
sys.path.insert(0, SOURCE)
from rover_drive_trace.trace import NullTrace, TracePublisher, timed_call


class MemoryTrace(NullTrace):
    enabled = True

    def __init__(self):
        self.events = []
        self.tick = 1.0

    def now(self):
        self.tick += .0001
        return self.tick

    def emit(self, event, fields=None, at=None):
        self.events.append((event, copy.deepcopy(fields)))


class Clock:
    def __init__(self, start=100):
        self.value = start

    def time(self):
        self.value += .001
        return self.value


class Stamp:
    def __init__(self, ns=123456):
        self.ns = ns

    def to_nsec(self):
        return self.ns


def twist(v, w):
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=v),
                                 angular=types.SimpleNamespace(z=w),
                                 _connection_header={'callerid': '/test'})


def wheel(v, w):
    return types.SimpleNamespace(twist=types.SimpleNamespace(twist=twist(v, w)),
                                 header=types.SimpleNamespace(stamp=Stamp(), seq=7))


class Environment:
    def __init__(self):
        self.calls = []
        self.responses = []
        self.fail_at = None
        self.rospy = types.ModuleType('rospy')
        self.rospy.ServiceException = type('ServiceException', (Exception,), {})
        self.rospy.ROSInterruptException = type('ROSInterruptException', (Exception,), {})
        self.rospy.ServiceProxy = lambda *args: self.service
        self.rospy.loginfo = lambda *args: None
        self.rospy.logwarn = lambda *args: None
        self.rospy.Time = types.SimpleNamespace(now=lambda: Stamp())
        self.rospy.get_name = lambda: '/pi1/test'
        self.parameters = {}
        self.rospy.get_param = lambda key, default: self.parameters.get(key, default)
        self.rospy.on_shutdown = lambda callback: None
        self.publishers = {}

        def publisher(name, *args, **kwargs):
            p = types.SimpleNamespace(messages=[], options=kwargs)
            p.publish = p.messages.append
            self.publishers[name] = p
            return p

        self.rospy.Publisher = publisher
        self.modules = {'rospy': self.rospy, 'tf': types.ModuleType('tf'),
                        'numpy': types.ModuleType('numpy')}
        for package, names in [('lightrover_ros.srv', ['Wrc201Msg']),
                               ('nav_msgs.msg', ['Odometry']),
                               ('geometry_msgs.msg', ['Twist', 'PoseStamped',
                                'TwistStamped', 'Quaternion', 'TransformStamped']),
                               ('std_msgs.msg', ['String'])]:
            parent = package.split('.')[0]
            self.modules[parent] = types.ModuleType(parent)
            module = types.ModuleType(package)
            for name in names:
                setattr(module, name, type(name, (), {
                    '__init__': lambda obj, **kw: obj.__dict__.update(kw)}))
            self.modules[package] = module

    def service(self, *args):
        self.calls.append(args)
        if len(self.calls) == self.fail_at:
            raise self.rospy.ServiceException('injected service failure')
        value = self.responses.pop(0) if self.responses else 1
        return types.SimpleNamespace(readData=value)

    def load(self, filename, upstream=False):
        path = os.path.join(ROOT, 'tests', 'upstream', filename) if upstream else \
            os.path.join(SOURCE, 'rover_drive_trace', filename)
        spec = importlib.util.spec_from_file_location('test_module', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, self.modules):
            spec.loader.exec_module(module)
        module.time = Clock()
        return module

    def controller(self, upstream=False, enabled=True):
        motor = self.load('vs_wrc201_motor.py' if upstream else 'motor.py', upstream)
        self.modules['vs_wrc201_motor'] = motor
        import rover_drive_trace
        # Each module/model gets independent class-level mutable arrays.
        with patch.object(rover_drive_trace, 'motor', motor, create=True):
            node = self.load('pos_controller.py' if upstream else 'controller.py', upstream)
        node.trace = MemoryTrace() if enabled else NullTrace()
        return node


class DriveRegressionTests(unittest.TestCase):
    def test_command_without_transport_header_is_still_accepted(self):
        node = Environment().controller()
        command = twist(.05, 0)
        command._connection_header = None
        node.cb_set_target_v(command)
        self.assertEqual(node.target_rover_v, [-.05, .05])
        self.assertIsNone(node.latest_command['caller_id'])

    def test_controller_matches_upstream_with_and_without_trace(self):
        for enabled in (True, False):
            original_env, traced_env = Environment(), Environment()
            original = original_env.controller(upstream=True)
            traced = traced_env.controller(enabled=enabled)
            sequence = [(0.05, 0)] * 12 + [(0, 0)] * 8 + \
                       [(-.05, 0)] * 12 + [(0, .2)] * 8 + [(0, 0)] * 8
            for i, (v, w) in enumerate(sequence):
                command = twist(v, w)
                observation = wheel(.01 if i % 3 else 0, .002)
                original.cb_set_target_v(command)
                traced.cb_set_target_v(command)
                original.cb_get_rover_v(observation)
                traced.cb_get_rover_v(observation)
                self.assertEqual(original.motor_controller.v_com,
                                 traced.motor_controller.v_com)
                self.assertEqual(original.motor_controller.buf_enc_com,
                                 traced.motor_controller.buf_enc_com)
            self.assertEqual(original_env.calls, traced_env.calls)
            self.assertEqual(len(traced_env.calls), 3 * len(sequence))
            if enabled:
                event = traced.trace.events[-1][1]
                self.assertTrue(event['success'])
                self.assertEqual(event['wheel_odom_stamp_ns'], 123456)
                self.assertEqual(event['command_at_start']['command_seq'], len(sequence))
                self.assertEqual(len(event['service_calls']), 3)
                self.assertGreater(event['integration_dt_s'], 0)

    def test_failed_write_does_not_issue_trigger_or_hide_exception(self):
        env = Environment()
        node = env.controller()
        env.fail_at = 2
        node.cb_set_target_v(twist(.05, 0))
        with self.assertRaises(env.rospy.ServiceException):
            node.cb_get_rover_v(wheel(0, 0))
        self.assertEqual([call[0] for call in env.calls], [0x48, 0x4c])
        cycle = node.trace.events[-1][1]
        self.assertFalse(cycle['success'])
        self.assertFalse(cycle['service_calls'][-1]['success'])

    def test_initialization_writes_match_upstream(self):
        nodes = []
        for upstream in (True, False):
            env = Environment()
            node = env.controller(upstream=upstream)
            env.rospy.init_node = lambda *args, **kwargs: None
            env.rospy.wait_for_service = lambda *args: None
            env.rospy.Subscriber = lambda *args: None
            env.rospy.spin = lambda: None
            node.create_trace = lambda *args: node.trace
            node.pos_cntrl()
            nodes.append(env.calls)
        self.assertEqual(nodes[0], nodes[1])
        self.assertEqual([call[0] for call in nodes[0]],
                         [0x10, 0x11, 0x20, 0x22, 0x30, 0x32, 0x10])

    def test_encoder_calculation_and_read_order_match_upstream(self):
        for enabled in (True, False):
            original_env, traced_env = Environment(), Environment()
            original = original_env.load('odometry.py', upstream=True)
            traced = traced_env.load('odometry.py')
            traced.trace = MemoryTrace() if enabled else NullTrace()
            original.pre_time = traced.pre_time = 99.0
            # Include standstill, reverse, and the upstream rejected-delta path.
            counts = [(0, 0), (-3, 3), (-6, 7), (-6, 7), (2, -1),
                      (2000000, -2000000), (2000003, -2000002)]
            for a, b in counts:
                original_env.responses = [a, b]
                traced_env.responses = [a, b]
                expected = original.calSpeed()
                actual = traced.calSpeed()
                self.assertEqual(expected, actual)
                original.cal_odometry(*expected)
                traced.cal_odometry(*actual)
                self.assertEqual((original.x, original.y, original.th),
                                 (traced.x, traced.y, traced.th))
            self.assertEqual(original_env.calls, traced_env.calls)
            self.assertEqual(len(traced_env.calls), 2 * len(counts))
            if enabled:
                event = traced.trace.events[-1][1]
                self.assertEqual(event['encoder_counts'], list(counts[-1]))
                self.assertEqual(len(event['service_calls']), 2)

    def test_failed_encoder_read_retains_original_return_behavior(self):
        env = Environment()
        node = env.load('odometry.py')
        node.trace = MemoryTrace()
        node.pre_time = 99
        env.fail_at = 2
        self.assertIsNone(node.calSpeed())
        sample = node.trace.events[-1][1]
        self.assertFalse(sample['success'])
        self.assertIsNone(sample['encoder_counts'])
        self.assertFalse(sample['service_calls'][-1]['success'])
        self.assertEqual(len(env.calls), 2)


class TransportTests(unittest.TestCase):
    def test_worker_publishes_without_ros_or_a_recording_subscriber(self):
        env = Environment()
        with patch.dict(sys.modules, env.modules):
            trace = TracePublisher(env.rospy, 'controller', {'test': True})
        try:
            trace.emit('test', {'value': 7})
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                messages = list(trace.publisher.messages)
                if any(json.loads(m.data)['events'] for m in messages):
                    break
                time.sleep(.01)
            self.assertTrue(any(json.loads(m.data)['events'] for m in messages))
            self.assertGreaterEqual(len(trace.metadata_publisher.messages), 1)
        finally:
            trace.close()
        self.assertFalse(trace.thread.is_alive())

    def make_trace(self, capacity=2):
        env = Environment()
        env.parameters['~trace_queue_capacity'] = capacity
        with patch.dict(sys.modules, env.modules):
            trace = TracePublisher(env.rospy, 'controller', {'test': True},
                                   clock=lambda: 12.5, start_worker=False)
        return env, trace

    def test_bounded_queue_reports_drops_and_preserves_event_time(self):
        env, trace = self.make_trace()
        for i in range(4):
            trace.emit('test', {'value': i}, at=11.0 + i)
        trace.flush()
        payload = json.loads(trace.publisher.messages[-1].data)
        self.assertEqual(payload['queue_dropped'], 2)
        self.assertEqual([r['seq'] for r in payload['events']], [3, 4])
        self.assertEqual(payload['events'][0]['monotonic_s'], 13.0)
        self.assertEqual(payload['events'][0]['ros_time_ns'], 123456)
        self.assertEqual(trace.publisher.options['queue_size'], 10)
        trace.publish_metadata()
        metadata = json.loads(trace.metadata_publisher.messages[-1].data)
        self.assertTrue(metadata['metadata']['test'])
        self.assertIn('controller.py', metadata['metadata']['source_sha256'])
        self.assertTrue(trace.metadata_publisher.options['latch'])

    def test_encoding_or_publish_failure_does_not_escape(self):
        env, trace = self.make_trace()
        trace.emit('bad', {'value': float('nan')})
        trace.flush()
        self.assertEqual(trace.publish_errors, 1)
        trace.publisher.publish = lambda msg: (_ for _ in ()).throw(RuntimeError('offline'))
        trace.emit('good', {'value': 1})
        trace.flush()
        self.assertEqual(trace.publish_errors, 2)

    def test_capture_failure_and_null_trace(self):
        env, trace = self.make_trace()
        trace.emit('bad', 42)
        self.assertEqual(trace.capture_errors, 1)
        null = NullTrace()
        self.assertIsNone(null.now())
        null.emit('ignored')
        env.responses = [3]
        self.assertEqual(timed_call(null, env.service, (0x60, 0, 4, 'r'), []).readData, 3)

    def test_concurrent_producers_keep_unique_sequences(self):
        env, trace = self.make_trace(capacity=400)
        threads = [threading.Thread(target=lambda: [trace.emit('test') for _ in range(100)])
                   for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual([r['seq'] for r in trace.queue], list(range(1, 401)))


class PackagingTests(unittest.TestCase):
    def test_upstream_fixtures_are_pinned_git_blobs(self):
        expected = {'pos_controller.py': '618bb67db1044e75fbde482903250f14aa94adb9',
                    'odometry.py': '8fd864156607c2a2a87455a27bea5b210ef1f1a7',
                    'vs_wrc201_motor.py': '599c2eedeec64bc68008ad2aedce1f45e3dfe8d3'}
        for name, sha in expected.items():
            with open(os.path.join(ROOT, 'tests', 'upstream', name), 'rb') as f:
                content = f.read().replace(b'\r\n', b'\n')
            blob = b'blob ' + str(len(content)).encode('ascii') + b'\0' + content
            self.assertEqual(hashlib.sha1(blob).hexdigest(), sha)

    def test_launch_has_exactly_one_controller_and_odometry_for_each_mode(self):
        path = os.path.join(ROOT, 'harada-tsubakino', 'launch', 'harada-tsubakino.launch')
        launch = ET.parse(path).getroot()
        for enabled in (True, False):
            nodes = [n for n in launch.findall('.//node')
                     if not ('if' in n.attrib and not enabled)
                     and not ('unless' in n.attrib and enabled)]
            for name in ('rover_pos', 'rover_odometry'):
                matches = [n for n in nodes if n.get('name') == name]
                self.assertEqual(len(matches), 1)
                self.assertEqual(matches[0].get('pkg'),
                                 'harada-tsubakino' if enabled else 'lightrover_ros')
                self.assertTrue(any(m.attrib == {'from': 'odom', 'to': 'odometry/wheel'}
                                    for m in matches[0].findall('remap')))


if __name__ == '__main__':
    unittest.main()

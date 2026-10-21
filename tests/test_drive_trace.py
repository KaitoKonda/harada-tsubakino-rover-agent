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
from rover_drive_trace.trace import (NullTrace, TracePublisher, timed_call, resolve_mode,
                                    create_trace, create_service_proxy)


class MemoryTrace(NullTrace):
    enabled = True
    detailed = True

    def __init__(self):
        self.events = []
        self.tick = 1.0

    def now(self):
        self.tick += .0001
        return self.tick

    def emit(self, event, fields=None, at=None):
        self.events.append((event, copy.deepcopy(fields)))


class MinimalMemoryTrace(MemoryTrace):
    detailed = False

    def emit(self, *args, **kwargs):
        raise AssertionError('minimal mode must not construct detailed events')


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
        self.lookup_calls = []
        self.transports = []
        env = self

        class ServiceProxy:
            def __init__(self, name, service_class, persistent=False, headers=None):
                self.name = name
                self.service_class = service_class
                self.persistent = persistent
                self.headers = headers
                self.transport = None

            def __call__(self, *args, **kwargs):
                return self.call(*args, **kwargs)

            def call(self, *args, **kwargs):
                request = types.SimpleNamespace(addr=args[0], data=args[1],
                                                length=args[2], cmd=args[3])
                transport = self.transport
                if transport is None:
                    self._get_service_uri(request)
                    transport = env.connect(request)
                    if self.persistent:
                        self.transport = transport
                try:
                    return env.service(*args, **kwargs)
                finally:
                    if not self.persistent:
                        transport.close()

            def _get_service_uri(self, request):
                return env.lookup(request)

            def close(self):
                if self.transport is not None:
                    self.transport.close()

        self.rospy.ServiceProxy = ServiceProxy
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

    def lookup(self, request):
        self.lookup_calls.append(request)
        return 'rosrpc://pi1:12345'

    def connect(self, request):
        transport = types.SimpleNamespace(closed=False, closes=0)

        def close():
            transport.closed = True
            transport.closes += 1

        transport.close = close
        self.transports.append(transport)
        return transport

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
        node.trace = (MinimalMemoryTrace() if enabled == 'minimal' else
                      MemoryTrace() if enabled else NullTrace())
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
        for enabled in (True, 'minimal', False):
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
            if enabled is True:
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
        for enabled in (True, 'minimal', False):
            original_env, traced_env = Environment(), Environment()
            original = original_env.load('odometry.py', upstream=True)
            traced = traced_env.load('odometry.py')
            traced.trace = (MinimalMemoryTrace() if enabled == 'minimal' else
                            MemoryTrace() if enabled else NullTrace())
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
            if enabled is True:
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


class ServiceLookupTests(unittest.TestCase):
    def make_trace(self, env, mode, clock):
        with patch.dict(sys.modules, env.modules):
            return TracePublisher(env.rospy, 'controller', {}, clock=clock,
                                  start_worker=False, mode=mode)

    def test_native_lookup_and_request_are_each_called_once_in_both_modes(self):
        for mode in ('full', 'minimal'):
            env = Environment()
            clock = [0.0]
            trace = self.make_trace(env, mode, lambda: clock[0])
            native_lookup, native_service = env.lookup, env.service

            def lookup(request):
                clock[0] += .025
                return native_lookup(request)

            def service(*args):
                clock[0] += .075
                return native_service(*args)

            env.lookup, env.service = lookup, service
            active_trace = [NullTrace()]
            proxy = create_service_proxy(env.rospy, 'wrc201_i2c', object,
                                         lambda: active_trace[0])
            # Nodes construct the proxy before initializing the real trace.
            active_trace[0] = trace
            self.assertFalse(proxy.persistent)
            self.assertIsNone(proxy.headers)
            self.assertNotIn('__call__', proxy.__class__.__dict__)
            records = [] if trace.detailed else None
            for args in ((0x48, 7, 4, 'w'), (0x60, 0, 4, 'r')):
                result = timed_call(trace, proxy, args, records)
                self.assertEqual(result.readData, 1)
            self.assertEqual(len(env.lookup_calls), 2)
            self.assertEqual(env.calls, [(0x48, 7, 4, 'w'), (0x60, 0, 4, 'r')])
            trace.flush_timing()
            metrics = json.loads(trace.timing_publisher.messages[-1].data)['metrics']
            for operation in ('w_0x48', 'r_0x60'):
                lookup = metrics['service_lookup_' + operation + '_s']
                total = metrics['service_' + operation + '_s']
                self.assertEqual(lookup['count'], 1)
                self.assertEqual(lookup['failures'], 0)
                self.assertAlmostEqual(lookup['mean'], .025)
                self.assertAlmostEqual(total['mean'], .1)

    def test_lookup_failure_preserves_exception_and_does_not_send_request(self):
        env = Environment()
        clock = [0.0]
        trace = self.make_trace(env, 'minimal', lambda: clock[0])
        failure = env.rospy.ServiceException('master unavailable')

        def lookup(request):
            clock[0] += .05
            raise failure

        env.lookup = lookup
        proxy = create_service_proxy(env.rospy, 'wrc201_i2c', object, lambda: trace)
        with self.assertRaises(env.rospy.ServiceException) as error:
            timed_call(trace, proxy, (0x11, 3, 1, 'w'), None)
        self.assertIs(error.exception, failure)
        self.assertEqual(env.calls, [])
        trace.flush_timing()
        metrics = json.loads(trace.timing_publisher.messages[-1].data)['metrics']
        self.assertEqual(metrics['service_lookup_w_0x11_s']['failures'], 1)
        self.assertEqual(metrics['service_w_0x11_s']['failures'], 1)
        self.assertAlmostEqual(metrics['service_lookup_w_0x11_s']['mean'], .05)

    def test_service_failure_after_successful_lookup_is_not_a_lookup_failure(self):
        env = Environment()
        trace = self.make_trace(env, 'minimal', lambda: 0.0)
        env.fail_at = 1
        proxy = create_service_proxy(env.rospy, 'wrc201_i2c', object, lambda: trace)
        with self.assertRaises(env.rospy.ServiceException):
            timed_call(trace, proxy, (0x48, 0, 4, 'w'), None)
        trace.flush_timing()
        metrics = json.loads(trace.timing_publisher.messages[-1].data)['metrics']
        self.assertEqual(metrics['service_lookup_w_0x48_s']['failures'], 0)
        self.assertEqual(metrics['service_w_0x48_s']['failures'], 1)
        self.assertEqual(len(env.calls), 1)

    def test_off_delegates_lookup_without_reading_the_clock(self):
        env = Environment()
        trace = NullTrace()
        trace.now = lambda: self.fail('off mode must not read the trace clock')
        proxy = create_service_proxy(env.rospy, 'wrc201_i2c', object, lambda: trace)
        self.assertEqual(timed_call(trace, proxy, (0x60, 0, 4, 'r'), None).readData, 1)
        self.assertEqual(len(env.lookup_calls), 1)
        self.assertEqual(env.calls, [(0x60, 0, 4, 'r')])

    def test_concurrent_lookup_timings_do_not_share_start_times(self):
        env = Environment()
        clock = threading.local()
        trace = self.make_trace(env, 'minimal', lambda: getattr(clock, 'value', 0.0))
        barrier = threading.Barrier(2, timeout=2)

        def lookup(request):
            clock.value += request.addr / 1000.0
            barrier.wait()
            return 'rosrpc://pi1:12345'

        env.lookup = lookup
        proxy = create_service_proxy(env.rospy, 'wrc201_i2c', object, lambda: trace)
        errors = []

        def call(address):
            try:
                clock.value = 0.0
                timed_call(trace, proxy, (address, 0, 4, 'r'), None)
            except Exception as error:
                errors.append(error)

        threads = [threading.Thread(target=call, args=(a,)) for a in (0x60, 0x64)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        trace.flush_timing()
        metrics = json.loads(trace.timing_publisher.messages[-1].data)['metrics']
        self.assertAlmostEqual(metrics['service_lookup_r_0x60_s']['mean'], .096)
        self.assertAlmostEqual(metrics['service_lookup_r_0x64_s']['mean'], .100)

    def test_master_uri_is_saved_in_metadata(self):
        env = Environment()
        with patch.dict(os.environ, {'ROS_MASTER_URI': 'http://control-pc:11311'}):
            trace = self.make_trace(env, 'minimal', lambda: 0.0)
        self.assertEqual(trace.metadata['ros_master_uri'], 'http://control-pc:11311')


class PersistentServiceTests(unittest.TestCase):
    def proxy(self, env, persistent=True, trace=None):
        return create_service_proxy(env.rospy, 'wrc201_i2c', object,
                                    lambda: trace or NullTrace(), persistent=persistent)

    def test_reuses_connection_and_only_records_lookup_on_first_call(self):
        for mode in ('full', 'minimal', 'off'):
            env = Environment()
            trace = (NullTrace() if mode == 'off' else
                ServiceLookupTests().make_trace(env, mode, lambda: 0.0))
            proxy = self.proxy(env, trace=trace)
            args = [(0x48, 12, 4, 'w'), (0x4c, -12, 4, 'w'), (0x11, 3, 1, 'w')]
            records = [] if trace.detailed else None
            for request in args:
                self.assertEqual(timed_call(trace, proxy, request, records).readData, 1)
            self.assertTrue(proxy.persistent)
            self.assertEqual(env.calls, args)
            self.assertEqual(len(env.lookup_calls), 1)
            self.assertEqual(len(env.transports), 1)
            self.assertFalse(env.transports[0].closed)
            if trace.enabled:
                trace.flush_timing()
                metrics = json.loads(trace.timing_publisher.messages[-1].data)['metrics']
                self.assertEqual(metrics['service_lookup_w_0x48_s']['count'], 1)
                self.assertNotIn('service_lookup_w_0x4c_s', metrics)
                self.assertNotIn('service_lookup_w_0x11_s', metrics)
                self.assertEqual(metrics['service_w_0x11_s']['count'], 1)

    def test_failure_does_not_retry_and_next_request_reconnects(self):
        env = Environment()
        proxy = self.proxy(env)
        proxy(0x48, 10, 4, 'w')
        original = proxy.transport
        env.fail_at = 2
        with self.assertRaises(env.rospy.ServiceException):
            proxy(0x4c, -10, 4, 'w')
        self.assertEqual(env.calls, [(0x48, 10, 4, 'w'), (0x4c, -10, 4, 'w')])
        self.assertTrue(original.closed)
        self.assertIsNone(proxy.transport)
        self.assertEqual(len(env.lookup_calls), 1)
        env.fail_at = None
        proxy(0x48, 20, 4, 'w')
        self.assertIsNot(proxy.transport, original)
        self.assertEqual(len(env.lookup_calls), 2)
        self.assertEqual(len(env.calls), 3)
        self.assertEqual(env.calls[-1], (0x48, 20, 4, 'w'))

    def test_cleanup_failure_preserves_original_exception_and_clears_transport(self):
        env = Environment()
        proxy = self.proxy(env)
        proxy(0x60, 0, 4, 'r')
        original = env.rospy.ServiceException('response lost after board accepted request')

        def fail(*args):
            env.calls.append(args)
            raise original

        def fail_close():
            raise OSError('socket already closed')

        env.service = fail
        proxy.transport.close = fail_close
        with self.assertRaises(env.rospy.ServiceException) as error:
            proxy(0x48, 25, 4, 'w')
        self.assertIs(error.exception, original)
        self.assertIsNone(proxy.transport)
        self.assertEqual(len(env.calls), 2)

    def test_initial_connect_failure_is_not_retried(self):
        env = Environment()
        proxy = self.proxy(env)
        connect = env.connect
        failure = env.rospy.ServiceException('connect failed')

        def fail(request):
            raise failure

        env.connect = fail
        with self.assertRaises(env.rospy.ServiceException) as error:
            proxy(0x60, 0, 4, 'r')
        self.assertIs(error.exception, failure)
        self.assertEqual(env.calls, [])
        self.assertEqual(len(env.lookup_calls), 1)
        self.assertIsNone(proxy.transport)
        env.connect = connect
        proxy(0x64, 0, 4, 'r')
        self.assertEqual(len(env.lookup_calls), 2)
        self.assertEqual(env.calls, [(0x64, 0, 4, 'r')])

    def test_close_discards_cached_transport_and_next_call_can_reconnect(self):
        env = Environment()
        proxy = self.proxy(env)
        proxy(0x60, 0, 4, 'r')
        transport = proxy.transport
        proxy.close()
        proxy.close()
        self.assertTrue(transport.closed)
        self.assertEqual(transport.closes, 1)
        self.assertIsNone(proxy.transport)
        proxy(0x64, 0, 4, 'r')
        self.assertEqual(len(env.transports), 2)

    def test_nonpersistent_reconnects_for_each_request(self):
        env = Environment()
        proxy = self.proxy(env, persistent=False)
        for _ in range(3):
            proxy(0x60, 0, 4, 'r')
        self.assertEqual(len(env.calls), 3)
        self.assertEqual(len(env.lookup_calls), 3)
        self.assertEqual(len(env.transports), 3)
        self.assertTrue(all(t.closed for t in env.transports))

    def test_persistent_calls_are_serialized(self):
        env = Environment()
        proxy = self.proxy(env)
        first_entered, release_first = threading.Event(), threading.Event()
        second_attempted, second_entered = threading.Event(), threading.Event()
        service = env.service
        errors = []

        def blocking_service(*args):
            if args[0] == 0x60:
                first_entered.set()
                if not release_first.wait(2):
                    raise AssertionError('test did not release first call')
            else:
                second_entered.set()
            return service(*args)

        env.service = blocking_service

        def call(address):
            try:
                if address == 0x64:
                    second_attempted.set()
                proxy(address, 0, 4, 'r')
            except Exception as error:
                errors.append(error)

        first = threading.Thread(target=call, args=(0x60,))
        second = threading.Thread(target=call, args=(0x64,))
        first.start()
        try:
            self.assertTrue(first_entered.wait(2))
            second.start()
            self.assertTrue(second_attempted.wait(2))
            self.assertFalse(second_entered.wait(.05))
        finally:
            release_first.set()
            first.join(3)
            if second.ident is not None:
                second.join(3)
        self.assertEqual(errors, [])
        self.assertFalse(first.is_alive())
        self.assertFalse(second.is_alive())
        self.assertTrue(second_entered.is_set())
        self.assertEqual(len(env.transports), 1)
        self.assertEqual(env.calls, [(0x60, 0, 4, 'r'), (0x64, 0, 4, 'r')])

    def test_invalid_boolean_does_not_silently_enable_persistence(self):
        env = Environment()
        for invalid in ('false', 'true', 0, 1, None):
            with self.assertRaises(ValueError):
                self.proxy(env, persistent=invalid)

    def test_node_startup_applies_parameter_and_records_actual_setting(self):
        for persistent in (False, True):
            for filename in ('controller.py', 'odometry.py'):
                env = Environment()
                node = env.controller() if filename == 'controller.py' else env.load(filename)
                env.parameters['~i2c_persistent'] = persistent
                env.rospy.init_node = lambda *args, **kwargs: None
                env.rospy.wait_for_service = lambda *args: None
                env.rospy.Subscriber = lambda *args: None
                env.rospy.spin = lambda: None
                env.rospy.Rate = lambda *args: None
                env.rospy.is_shutdown = lambda: True
                env.modules['tf'].TransformBroadcaster = lambda: None
                captured = []

                def create_trace(rospy, stream, metadata):
                    captured.append(metadata)
                    return NullTrace()

                node.create_trace = create_trace
                if filename == 'controller.py':
                    node.pos_cntrl()
                    proxy = node.write_msg
                else:
                    node.lightrover_odometry()
                    proxy = node.read_enc
                self.assertEqual(proxy.persistent, persistent)
                self.assertEqual(captured[0]['service_proxy_persistent'], persistent)
                self.assertEqual(len(env.lookup_calls), 1 if persistent else len(env.calls))
                self.assertTrue(env.calls)

    def test_failed_persistent_controller_write_does_not_issue_trigger(self):
        env = Environment()
        node = env.controller()
        node.write_msg = self.proxy(env, trace=node.trace)
        env.fail_at = 2
        with self.assertRaises(env.rospy.ServiceException):
            node.cb_get_rover_v(wheel(0, 0))
        self.assertEqual([call[0] for call in env.calls], [0x48, 0x4c])
        self.assertIsNone(node.write_msg.transport)


class PackagingTests(unittest.TestCase):
    def test_launch_passes_string_mode_and_interval_to_both_nodes(self):
        path = os.path.join(ROOT, 'harada-tsubakino', 'launch', 'harada-tsubakino.launch')
        launch = ET.parse(path).getroot()
        defaults = {a.get('name'): a.get('default') for a in launch.findall('arg')}
        self.assertEqual(defaults['trace_mode'], '')
        self.assertEqual(defaults['trace_enabled'], 'true')
        self.assertEqual(defaults['trace_summary_interval'], '5.0')
        self.assertEqual(defaults['i2c_persistent'], 'false')
        for node in launch.findall('.//node'):
            if node.get('type') not in ('tracedOdometry.py', 'tracedPosController.py'):
                continue
            params = {p.get('name'): p.attrib for p in node.findall('param')}
            self.assertEqual(params['i2c_persistent']['type'], 'bool')
            self.assertEqual(params['i2c_persistent']['value'], '$(arg i2c_persistent)')
            self.assertEqual(params['trace_mode']['type'], 'str')
            self.assertEqual(params['trace_mode']['value'], '$(arg trace_mode)')
            self.assertEqual(params['trace_summary_interval']['value'],
                             '$(arg trace_summary_interval)')

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


class TimingModeTests(unittest.TestCase):
    def make_trace(self, mode, clock=None, env=None):
        env = env or Environment()
        with patch.dict(sys.modules, env.modules):
            trace = TracePublisher(env.rospy, 'controller', {},
                clock=clock or (lambda: 10.0), start_worker=False, mode=mode)
        return env, trace

    def test_mode_selection_legacy_compatibility_and_precedence(self):
        self.assertEqual(resolve_mode(), 'full')
        self.assertEqual(resolve_mode('', False), 'off')
        self.assertEqual(resolve_mode('', 'false'), 'off')
        self.assertEqual(resolve_mode('', 'true'), 'full')
        for mode in ('full', 'minimal', 'off'):
            self.assertEqual(resolve_mode(mode, False), mode)
            self.assertEqual(resolve_mode(mode, True), mode)
        for invalid in ('true', 'false', 'minmal', True, 1):
            with self.assertRaises(ValueError):
                resolve_mode(invalid)
        with self.assertRaises(ValueError):
            resolve_mode('', 'minimal')

    def test_off_creates_no_publisher_clock_or_worker(self):
        env = Environment()
        env.parameters['~trace_mode'] = 'off'
        with patch('rover_drive_trace.trace.make_monotonic') as clock:
            trace = create_trace(env.rospy, 'controller', {})
        self.assertFalse(trace.enabled)
        self.assertFalse(trace.detailed)
        self.assertEqual(env.publishers, {})
        clock.assert_not_called()
        env.parameters['~trace_mode'] = 'typo'
        with self.assertRaises(ValueError):
            create_trace(env.rospy, 'controller', {})

    def test_minimal_has_no_event_queue_or_detailed_publisher(self):
        env, trace = self.make_trace('minimal')
        self.assertTrue(trace.enabled)
        self.assertFalse(trace.detailed)
        self.assertIsNone(trace.queue)
        self.assertIsNone(trace.publisher)
        self.assertNotIn('drive_trace/controller', env.publishers)
        # Even an unserializable detail is ignored, without copying or encoding.
        trace.emit('ignored', object())
        trace.flush()
        self.assertEqual(trace.sequence, 0)
        self.assertEqual(trace.capture_errors, 0)
        env.responses = [4]
        with patch('rover_drive_trace.trace.json.dumps', side_effect=AssertionError('JSON on producer')):
            self.assertEqual(timed_call(trace, env.service, (0x60, 0, 4, 'r'), None).readData, 4)
        self.assertEqual(trace.timing_metrics['service_r_0x60_s'][0], 1)
        trace.flush_timing()
        summary = json.loads(trace.timing_publisher.messages[-1].data)
        self.assertEqual(summary['trace_mode'], 'minimal')
        self.assertEqual(summary['metrics']['service_r_0x60_s']['count'], 1)

    def test_full_and_minimal_have_identical_controller_and_wheel_summaries(self):
        summaries = []
        for mode in ('full', 'minimal'):
            env = Environment()
            node = env.controller()
            clock = [10.0]
            _, trace = self.make_trace(mode, clock=lambda: clock[0], env=env)
            node.trace = trace

            def service(*args):
                clock[0] += .01
                return env.service(*args)

            node.write_msg = service
            for v in (.05, 0, -.05):
                node.cb_set_target_v(twist(v, 0))
                clock[0] += .1
                node.cb_get_rover_v(wheel(0, 0))
            odometry = env.load('odometry.py')
            odometry.trace = trace
            odometry.pre_time = 99
            odometry.read_enc = service
            for _ in range(2):
                clock[0] += .1
                env.responses = [3, -3]
                odometry.calSpeed()
            trace.flush_timing()
            summary = json.loads(trace.timing_publisher.messages[-1].data)
            self.assertEqual(summary['counters']['control_cycles'], 3)
            self.assertEqual(summary['counters']['wheel_samples'], 2)
            self.assertEqual(summary['counters']['commands_received'], 3)
            self.assertEqual(summary['metrics']['control_duration_s']['count'], 3)
            self.assertEqual(summary['metrics']['control_period_s']['count'], 2)
            self.assertAlmostEqual(summary['metrics']['control_duration_s']['mean'], .03)
            summaries.append((summary['metrics'], summary['counters'], env.calls))
            if mode == 'minimal':
                self.assertIsNone(node.latest_command)
                self.assertIsNone(odometry.read_records)
                self.assertIsNone(odometry.last_read_counts)
        self.assertEqual(summaries[0], summaries[1])

    def test_minimal_failed_service_is_counted_and_still_raises(self):
        env, trace = self.make_trace('minimal')
        env.fail_at = 1
        with self.assertRaises(env.rospy.ServiceException):
            timed_call(trace, env.service, (0x48, 0, 4, 'w'), None)
        trace.flush_timing()
        summary = json.loads(trace.timing_publisher.messages[-1].data)
        self.assertEqual(summary['metrics']['service_w_0x48_s']['failures'], 1)
        self.assertEqual(len(env.calls), 1)

    def test_minimal_ignores_full_only_settings(self):
        env = Environment()
        env.parameters.update({'~trace_queue_capacity': 'unused',
            '~trace_batch_size': 'unused', '~trace_publish_hz': 'unused'})
        _, trace = self.make_trace('minimal', env=env)
        self.assertEqual(trace.mode, 'minimal')
        self.assertIsNone(trace.queue)

    def test_failed_control_and_wheel_cycles_have_common_summaries(self):
        for mode in ('full', 'minimal'):
            env = Environment()
            node = env.controller()
            _, trace = self.make_trace(mode, env=env)
            node.trace = trace
            env.fail_at = 2
            with self.assertRaises(env.rospy.ServiceException):
                node.cb_get_rover_v(wheel(0, 0))
            self.assertEqual([call[0] for call in env.calls], [0x48, 0x4c])
            odometry = env.load('odometry.py')
            odometry.trace = trace
            odometry.pre_time = 99
            env.fail_at = 4
            self.assertIsNone(odometry.calSpeed())
            trace.flush_timing()
            summary = json.loads(trace.timing_publisher.messages[-1].data)
            self.assertEqual(summary['counters']['control_failures'], 1)
            self.assertEqual(summary['counters']['wheel_sample_failures'], 1)
            self.assertEqual(summary['metrics']['service_w_0x4c_s']['failures'], 1)
            self.assertEqual(summary['metrics']['service_r_0x64_s']['failures'], 1)

    def test_summary_window_reset_and_invalid_values(self):
        clock = [10.0]
        _, trace = self.make_trace('minimal', clock=lambda: clock[0])
        trace.observe('duration_s', 1)
        trace.observe('duration_s', 2, failed=True)
        trace.observe('duration_s', None)
        trace.observe('duration_s', float('nan'))
        trace.count('cycles')
        clock[0] = 15.0
        trace.flush_timing()
        summary = json.loads(trace.timing_publisher.messages[-1].data)
        self.assertEqual(summary['metrics']['duration_s'],
            dict(count=2, sum=3, mean=1.5, min=1, max=2, first=1, last=2, failures=1))
        self.assertEqual(summary['window_duration_s'], 5)
        self.assertEqual(summary['window_start_monotonic_s'], 10)
        self.assertEqual(summary['capture_errors'], 1)
        self.assertEqual(summary['counters'], {'cycles': 1})
        self.assertEqual(trace.timing_metrics, {})
        self.assertEqual(trace.timing_counters, {})
        clock[0] = 17
        trace.flush_timing(partial=True)
        partial = json.loads(trace.timing_publisher.messages[-1].data)
        self.assertTrue(partial['partial'])
        self.assertEqual(partial['window_duration_s'], 2)
        self.assertEqual(partial['summary_seq'], 2)
        self.assertEqual(partial['metrics'], {})

    def test_minimal_worker_publishes_summary_and_stops_promptly(self):
        env = Environment()
        env.parameters['~trace_summary_interval'] = .02
        with patch.dict(sys.modules, env.modules):
            trace = TracePublisher(env.rospy, 'controller', {}, mode='minimal')
        try:
            trace.observe('test_s', .1)
            deadline = time.monotonic()+2
            summaries = []
            while time.monotonic()<deadline:
                summaries = [json.loads(m.data) for m in list(trace.timing_publisher.messages)]
                if any('test_s' in s['metrics'] for s in summaries):
                    break
                time.sleep(.005)
            self.assertTrue(trace.timing_publisher.messages)
            self.assertTrue(any('test_s' in s['metrics'] for s in summaries))
            self.assertNotIn('drive_trace/controller', env.publishers)
        finally:
            trace.close()
        self.assertFalse(trace.thread.is_alive())
        self.assertTrue(json.loads(trace.timing_publisher.messages[-1].data)['partial'])


if __name__ == '__main__':
    unittest.main()

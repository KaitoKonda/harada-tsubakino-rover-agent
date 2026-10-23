"""A-C output probe regression tests; no ROS, SMBus or motors required."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'harada-tsubakino' / 'src'))
from rover_drive_trace.motor_output import (MotorOutput, OutputError, ENABLE, WDT,
    GAIN, PUNCH, LIMIT, OFFSET, TARGET, POSITION, OUTPUT, signed, load_snapshot)
from rover_drive_trace.output_probe import OutputProbe, ProbeSettings, ConflictMonitor
from rover_drive_trace import output_probe
from rover_drive_trace import trace as trace_module


class Clock:
    def __init__(self):
        self.value = 100.

    def __call__(self):
        return self.value

    def advance(self, value):
        self.value += value


class Trace:
    enabled = detailed = True
    capture_errors = publish_errors = dropped = 0

    def __init__(self, clock):
        self.clock = clock
        self.events = []

    def now(self):
        return self.clock()

    def emit(self, name, fields):
        self.events.append((name, copy.deepcopy(fields)))

    def observe(self, *args, **kwargs):
        pass

    def close(self):
        self.closed = True

    def flush(self):
        pass


class Device:
    def __init__(self, clock):
        self.clock = clock
        self.regs = {ENABLE: 0, WDT: 0, 0: 0x21, 2: 7}
        for pair, value in ((GAIN, 128), (PUNCH, 2500), (LIMIT, 4096),
                            (OFFSET, 0), (TARGET, 30), (POSITION, 7)):
            self.regs.update((address, value) for address in pair)
        self.calls = []
        self.fail = None
        self.delay = None
        self.watchdog_until = None

    def close(self):
        self.closed = True

    def __call__(self, address, data, length, operation):
        self.calls.append((address, data, length, operation))
        self.clock.advance(.0001)
        if self.delay is not None:
            self.delay(address, operation)
        if self.fail is not None and self.fail(address, data, operation):
            raise IOError('injected I2C failure')
        if self.watchdog_until is not None and self.clock() >= self.watchdog_until:
            self.regs[ENABLE] = 0
        if operation == 'w':
            if address in OFFSET and data and self.regs[ENABLE]:
                raise AssertionError('live multi-byte T_OUT write')
            self.regs[address] = data & ((1 << (length * 8)) - 1)
            if address == WDT:
                self.watchdog_until = (self.clock() + data * .001) if 0 < data < 65535 else None
            return types.SimpleNamespace(readData=1)
        if address in OUTPUT:
            index = OUTPUT.index(address)
            value = self.regs[OFFSET[index]] if self.regs[ENABLE] else 0
        else:
            value = self.regs.get(address, 0)
        return types.SimpleNamespace(readData=value)


def settings(**updates):
    data = dict(output_limits=[100, 100], wheels_lifted=True, period_s=.05,
                command_timeout_s=.2, watchdog_ms=500, sensor_timeout_s=.3,
                max_duration_s=5., watchdog_wait_s=.8,
                sequence=[dict(duration_s=.2, t_out=[40, -40]),
                          dict(duration_s=.2, t_out=[0, 0])])
    data.update(updates)
    return ProbeSettings(data)


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = os.path.join(self.directory.name, 'snapshot.json')
        self.clock = Clock()
        self.trace = Trace(self.clock)
        self.device = Device(self.clock)
        self.original = dict(self.device.regs)
        self.board = MotorOutput(self.device, self.trace)
        self.probe = OutputProbe(self.board, settings(), self.clock)

    def ready(self, **updates):
        self.probe = OutputProbe(self.board, settings(**updates), self.clock)
        self.probe.prepare(self.path, 'pi1', dict(boot_id='boot', run_id='run'))
        return self.probe

    def start(self):
        self.probe.sensor_received(self.clock())
        self.assertTrue(self.probe.request_start()[0])
        self.probe.tick()

    def step(self, dt=.05):
        self.clock.advance(dt)
        self.probe.sensor_received(self.clock())
        self.probe.tick()

    def events(self, name):
        return [fields for event, fields in self.trace.events if event == name]

    def test_defaults_and_nonfinite_invalid_before_board_access(self):
        for data in ({}, dict(output_limits=[0, 1]),
                     dict(period_s=float('nan')), dict(period_s=float('inf')),
                     dict(sequence=[dict(duration_s=.1, t_out=[True, 0])]),
                     dict(wheels_lifted=False), dict(command_timeout_s=.6),
                     dict(max_duration_s=.1), dict(sequence=[])):
            with self.subTest(data=data), self.assertRaises(ValueError):
                settings(**data) if data else ProbeSettings({})
        self.assertEqual(self.device.calls, [])

    def test_snapshot_and_signed_roundtrip_leave_output_disabled(self):
        self.ready()
        snapshot = load_snapshot(self.path, 'pi1')
        self.assertEqual(snapshot['registers'][str(GAIN[0])], 128)
        self.assertEqual(self.device.regs[ENABLE], 0)
        self.assertEqual([self.device.regs[a] for a in OFFSET], [0, 0])
        self.assertEqual(self.probe.state, 'READY')
        self.probe.tick()
        self.assertFalse(any(a == ENABLE and d == 3 for a, d, _, _ in self.device.calls))
        for address in OFFSET:
            self.assertIn((address, -1, 2, 'w'), self.device.calls)

    def test_output_layer_independently_rejects_unconfigured_or_unbounded_input(self):
        with self.assertRaises(OutputError):
            self.board.apply([1, 1], 500, self.clock)
        self.assertEqual(self.device.calls, [])
        self.ready()
        before = len(self.device.calls)
        for values, wdt in (([101, 0], 500), ([0], 500), ([0, 0], 0)):
            with self.assertRaises(ValueError):
                self.board.apply(values, wdt, self.clock)
        self.assertEqual(len(self.device.calls), before)

    def test_snapshot_is_never_overwritten(self):
        with open(self.path, 'w') as handle:
            handle.write('existing recovery record')
        with self.assertRaises(OSError):
            self.ready()
        with open(self.path) as handle:
            self.assertEqual(handle.read(), 'existing recovery record')
        self.assertEqual(self.probe.state, 'FAULT')
        self.assertFalse(any(a in GAIN and d == 0 for a, d, _, op in self.device.calls if op == 'w'))

    def test_conflict_before_prepare_never_touches_board(self):
        def conflict():
            raise OutputError('normal controller still running')
        self.probe.conflict_check = conflict
        with self.assertRaises(OutputError):
            self.probe.prepare(self.path, 'pi1', {})
        self.probe.finish('startup exception')
        self.assertEqual(self.device.calls, [])

    def test_full_trace_required_before_any_i2c(self):
        self.trace.detailed = False
        with self.assertRaises(OutputError):
            self.probe.prepare(self.path, 'pi1', {})
        self.probe.finish('no trace')
        self.assertEqual(self.device.calls, [])

    def test_finite_sequence_stops_and_restores_with_disabled_output(self):
        self.ready()
        self.start()
        self.assertEqual(self.probe.state, 'RUNNING')
        self.assertEqual(self.events('output_readback')[-1]['m_out_signed'], [40, -40])
        for _ in range(10):
            self.step()
        self.assertTrue(self.probe.finished)
        self.assertFalse(self.probe.failed)
        self.assertEqual(self.device.regs[ENABLE], 0)
        self.assertEqual(self.device.regs[WDT], 0)
        for pair in (GAIN, PUNCH, LIMIT, TARGET):
            self.assertEqual([self.device.regs[a] for a in pair], [self.original[a] for a in pair])
        self.assertTrue(all(e['update_completed'] for e in self.events('output_command')))
        self.assertFalse(self.probe.request_start()[0])

    def test_second_wheel_failure_does_not_refresh_watchdog_or_enable(self):
        self.ready()
        self.device.fail = lambda a, d, op: a == OFFSET[1] and d == -40 and op == 'w'
        self.start()
        self.assertEqual(self.probe.state, 'FAULT')
        commands = self.events('output_command')
        self.assertFalse(commands[-1]['update_completed'])
        self.assertFalse(any(a == ENABLE and d == 3 for a, d, _, _ in self.device.calls))
        self.assertFalse(any(a == WDT and d > 0 for a, d, _, _ in self.device.calls))

    def test_stale_sensor_stops_without_renewing_lease(self):
        self.ready()
        self.start()
        before = len(self.device.calls)
        self.clock.advance(.31)
        self.probe.tick()
        self.assertTrue(self.probe.failed)
        writes = [(a, d) for a, d, _, op in self.device.calls[before:] if op == 'w']
        self.assertNotIn((ENABLE, 3), writes)
        self.assertFalse(any(a == WDT and d > 0 for a, d in writes))

    def test_missing_sensor_prevents_first_enable(self):
        self.ready()
        self.probe.request_start()
        self.probe.tick()
        self.assertTrue(self.probe.failed)
        self.assertFalse(any(a == ENABLE and d == 3 for a, d, _, _ in self.device.calls))

    def test_inflight_stop_prevents_enable_after_offset_write(self):
        self.ready()
        self.device.delay = lambda a, op: self.probe.request_stop() if a == OFFSET[0] and op == 'w' else None
        self.start()
        self.assertTrue(self.probe.failed)
        self.assertFalse(any(a == ENABLE and d == 3 for a, d, _, _ in self.device.calls))

    def test_inflight_deadline_prevents_watchdog_refresh(self):
        self.ready()
        delayed = [False]
        def delay(a, op):
            if a == OFFSET[0] and op == 'w' and not delayed[0]:
                delayed[0] = True
                self.clock.advance(.25)
        self.device.delay = delay
        self.start()
        self.assertTrue(self.probe.failed)
        self.assertFalse(any(a == WDT and d > 0 for a, d, _, _ in self.device.calls))

    def test_configuration_changes_are_detected_before_output(self):
        self.ready()
        self.device.regs[GAIN[0]] = 1
        self.start()
        self.assertTrue(self.probe.failed)
        self.assertFalse(any(a == ENABLE and d == 3 for a, d, _, _ in self.device.calls))

    def test_log_failure_stops_experiment(self):
        self.ready()
        self.start()
        self.trace.publish_errors = 1
        self.step()
        self.assertTrue(self.probe.failed)
        self.assertEqual(self.device.regs[ENABLE], 0)

    def test_excessive_computed_output_faults(self):
        self.ready()
        service = self.board.service
        def excessive(a, d, n, op):
            response = service(a, d, n, op)
            if a == OUTPUT[0] and op == 'r':
                response.readData = 101
            return response
        self.board.service = excessive
        self.start()
        self.assertTrue(self.probe.failed)
        self.assertEqual(self.device.regs[ENABLE], 0)

    def test_conflict_appearing_during_run_stops_output(self):
        self.ready()
        self.start()
        def conflict():
            raise OutputError('new controller appeared')
        self.probe.conflict_check = conflict
        self.step()
        self.assertTrue(self.probe.failed)
        self.assertEqual(self.device.regs[ENABLE], 0)

    def run_main(self, params, graph=None):
        """Exercise lazy ROS integration and cleanup with no ROS installation."""
        rospy = types.ModuleType('rospy')
        rospy.init_node = lambda name: None
        rospy.get_param = lambda name, default=None: params if name == '~' else params.get(name[1:], default)
        rospy.get_namespace = lambda: '/pi1/'
        rospy.get_name = lambda: '/pi1/rover_output_probe'
        rospy.resolve_name = lambda name: '/pi1/' + name
        rospy.wait_for_service = lambda *args, **kwargs: None
        rospy.on_shutdown = lambda cb: None
        rospy.Subscriber = lambda *args, **kwargs: object()
        rospy.Service = lambda *args, **kwargs: object()
        rospy.loginfo = lambda *args: None
        rospy.is_shutdown = lambda: True
        rosgraph = types.ModuleType('rosgraph')
        rosgraph.Master = lambda name: types.SimpleNamespace(
            getSystemState=lambda: graph or ([], [], []))
        modules = {'rospy': rospy, 'rosgraph': rosgraph}
        for module, names in (('nav_msgs.msg', ['Odometry']),
                              ('std_srvs.srv', ['Trigger', 'TriggerResponse']),
                              ('lightrover_ros.srv', ['Wrc201Msg'])):
            modules[module] = types.ModuleType(module)
            modules[module.split('.')[0]] = types.ModuleType(module.split('.')[0])
            for name in names:
                setattr(modules[module], name, type(name, (), {}))
        self.trace.run_id = 'main-run'
        lock = types.SimpleNamespace(closed=False)
        lock.close = lambda: setattr(lock, 'closed', True)
        with patch.dict(sys.modules, modules), \
                patch.object(output_probe, 'ProbeLock', return_value=lock), \
                patch.object(trace_module, 'create_trace', return_value=self.trace), \
                patch.object(trace_module, 'make_monotonic', return_value=self.clock), \
                patch.object(trace_module, 'create_service_proxy', return_value=self.device):
            try:
                output_probe.main()
            finally:
                self.assertTrue(lock.closed)
                self.assertTrue(self.trace.closed)

    def test_ros_idle_shutdown_restores_and_closes_resources(self):
        params = settings().metadata()
        params['snapshot_path'] = self.path
        self.run_main(params)
        self.assertEqual(self.device.regs[ENABLE], 0)
        self.assertEqual(self.device.regs[GAIN[0]], 128)
        self.assertTrue(self.device.closed)

    def test_ros_conflict_never_creates_a_board_write(self):
        with self.assertRaises(OutputError):
            self.run_main(settings().metadata(), graph=([], [('/pi1/rover_drive', ['/pi1/rover_pos'])], []))
        self.assertEqual(self.device.calls, [])

    def test_ros_restore_mode_does_not_require_a_motion_sequence(self):
        self.ready()
        self.run_main(dict(restore_snapshot=self.path))
        self.assertEqual(self.device.regs[ENABLE], 0)
        self.assertEqual(self.device.regs[GAIN[0]], 128)

    def test_watchdog_observation_does_not_write_until_restore(self):
        self.ready(experiment='watchdog', sequence=[dict(duration_s=.15, t_out=[20, 0])])
        self.start()
        while self.probe.watchdog_observation_started is None:
            self.step(.05)
        before = len(self.device.calls)
        self.step(.1)
        self.step(.1)
        self.assertTrue(all(op == 'r' for _, _, _, op in self.device.calls[before:]))
        while not self.probe.finished:
            self.step(.05)
        self.assertFalse(self.probe.failed)
        result = self.events('output_watchdog_result')[-1]
        self.assertTrue(result['computed_zero_observed'])
        self.assertFalse(result['physical_stop_verified'])

    def test_watchdog_failure_is_reported(self):
        self.ready(experiment='watchdog', sequence=[dict(duration_s=.15, t_out=[20, 0])])
        self.start()
        for _ in range(3):
            self.step(.05)
        # Simulate a device that never expires its hardware timer.
        self.device.watchdog_until = None
        while not self.probe.finished:
            self.step(.05)
        self.assertTrue(self.probe.failed)
        self.assertFalse(self.events('output_watchdog_result')[-1]['computed_zero_observed'])

    def test_position_offsets_are_written_only_once_per_segment_and_restored(self):
        self.ready(experiment='position', sequence=[dict(duration_s=.2, t_out=[20, 0],
                                                        target_offset_counts=[12, -12])])
        self.start()
        self.step(.05)
        self.assertEqual([signed(self.device.regs[a], 32) for a in TARGET], [19, -5])
        self.probe.request_stop()
        self.probe.tick()
        self.assertEqual([self.device.regs[a] for a in TARGET], [30, 30])

    def test_failed_disable_prevents_gain_restore_and_watchdog_disarm(self):
        self.ready()
        self.start()
        self.device.fail = lambda a, d, op: a == ENABLE and op == 'w'
        before = len(self.device.calls)
        self.probe.request_stop()
        self.probe.tick()
        writes = [(a, d) for a, d, _, op in self.device.calls[before:] if op == 'w']
        self.assertTrue(self.probe.failed)
        self.assertFalse(any(a in GAIN or (a == WDT and d == 0) for a, d in writes))

    def test_original_nonzero_offset_and_enable_are_not_restored(self):
        self.device.regs[OFFSET[0]] = 70
        self.device.regs[ENABLE] = 3
        self.ready()
        self.probe.finish('done')
        self.assertEqual(self.device.regs[ENABLE], 0)
        self.assertEqual(self.device.regs[OFFSET[0]], 0)

    def test_restore_rejects_wrong_vehicle_registers_and_board_identity(self):
        self.ready()
        with self.assertRaises(OutputError):
            load_snapshot(self.path, 'pi2')
        snapshot = load_snapshot(self.path, 'pi1')
        self.device.regs[2] = 99
        with self.assertRaises(OutputError):
            self.board.use_snapshot(snapshot)
        snapshot['registers']['123'] = 0
        with open(self.path, 'w') as handle:
            json.dump(snapshot, handle)
        with self.assertRaises(OutputError):
            load_snapshot(self.path, 'pi1')

    def test_clock_reversal_faults_without_reenabling(self):
        self.ready()
        self.start()
        self.clock.advance(-1)
        self.probe.tick()
        self.assertTrue(self.probe.failed)

    def test_graph_monitor_latches_failure_and_rejects_stale_results(self):
        monitor = ConflictMonitor(lambda: None, self.clock)
        try:
            monitor.check()
            self.clock.advance(2.1)
            with self.assertRaises(OutputError):
                monitor.check()
            monitor.poll()
            monitor.check()
            def conflict():
                raise OutputError('competing writer')
            monitor.inspect_graph = conflict
            monitor.poll()
            monitor.inspect_graph = lambda: None
            monitor.poll()
            with self.assertRaises(OutputError):
                monitor.check()
        finally:
            monitor.close()

    def test_launch_never_includes_position_controller(self):
        root = ET.parse(ROOT / 'harada-tsubakino/launch/motor-output-identification.launch').getroot()
        types_ = [node.attrib['type'] for node in root.iter('node')]
        self.assertNotIn('tracedPosController.py', types_)
        self.assertNotIn('pos_controller.py', types_)
        args = {arg.attrib['name']: arg.attrib.get('default') for arg in root.findall('arg')}
        self.assertEqual(args['start_i2c'], 'false')
        self.assertEqual(args['start_wheel'], 'false')


if __name__ == '__main__':
    unittest.main()

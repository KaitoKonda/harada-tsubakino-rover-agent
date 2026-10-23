"""Finite, explicitly started A-C experiments, independent of ROS callbacks.

Only the worker calls I2C. Callbacks set start/stop flags or sensor timestamps.
"""
import hashlib
import os
import threading

from rover_drive_trace.motor_output import (MotorOutput, OutputError, finite,
                                            integer, load_snapshot)


def pair(value, name, lower, upper):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('%s needs motor0,motor1' % name)
    return tuple(integer(v, lower, upper, name) for v in value)


class ProbeSettings(object):
    def __init__(self, data):
        self.limits = pair(data.get('output_limits', [0, 0]), 'output_limits', 1, 4096)
        self.period = finite(data.get('period_s', 0.1), 'period_s')
        self.watchdog_ms = integer(data.get('watchdog_ms', 500), 1, 65534, 'watchdog_ms')
        self.timeout = finite(data.get('command_timeout_s', 0.3), 'command_timeout_s')
        self.sensor_timeout = finite(data.get('sensor_timeout_s', 0.3), 'sensor_timeout_s')
        self.max_duration = finite(data.get('max_duration_s', 20), 'max_duration_s')
        self.watchdog_wait = finite(data.get('watchdog_wait_s', 1), 'watchdog_wait_s')
        self.experiment = data.get('experiment', 'pulse')
        if self.experiment not in ('pulse', 'position', 'watchdog'):
            raise ValueError('experiment must be pulse, position or watchdog')
        if not (0 < self.period < self.timeout < self.watchdog_ms * 0.001 and
                self.sensor_timeout > 0 and self.max_duration > 0):
            raise ValueError('require 0 < period < command_timeout < watchdog; sensor/max > 0')
        if self.watchdog_wait <= self.watchdog_ms * 0.001:
            raise ValueError('watchdog_wait_s must exceed watchdog duration')
        # This release is the lifted-wheel A-C probe, not an on-floor controller.
        if data.get('wheels_lifted') is not True:
            raise ValueError('A-C probe requires wheels_lifted: true')
        rows = data.get('sequence', [])
        if not isinstance(rows, list) or not rows:
            raise ValueError('an explicit finite sequence is required')
        self.sequence = []
        for row in rows:
            duration = finite(row.get('duration_s'), 'duration_s')
            values = pair(row.get('t_out'), 't_out', -4096, 4096)
            offsets = pair(row.get('target_offset_counts', [0, 0]),
                           'target_offset_counts', -4096, 4096)
            if duration < self.period or any(abs(v) > cap for v, cap in zip(values, self.limits)):
                raise ValueError('segment too short or t_out exceeds explicit limit')
            if self.experiment != 'position' and any(offsets):
                raise ValueError('target offsets require the position experiment')
            self.sequence.append(dict(duration_s=duration, t_out=values,
                                      target_offset_counts=offsets))
        self.total = sum(row['duration_s'] for row in self.sequence)
        if self.experiment == 'watchdog':
            if len(self.sequence) != 1 or not any(self.sequence[0]['t_out']):
                raise ValueError('watchdog experiment needs one nonzero segment')
            self.total += self.watchdog_wait
        if self.total >= self.max_duration:
            raise ValueError('sequence including watchdog wait must be shorter than max_duration_s')

    def metadata(self):
        return dict(output_limits=list(self.limits), period_s=self.period,
                    watchdog_ms=self.watchdog_ms, command_timeout_s=self.timeout,
                    sensor_timeout_s=self.sensor_timeout, max_duration_s=self.max_duration,
                    watchdog_wait_s=self.watchdog_wait, experiment=self.experiment,
                    sequence=self.sequence, wheels_lifted=True)


class OutputProbe(object):
    def __init__(self, board, settings, clock, conflict_check=lambda: None):
        self.board, self.settings, self.clock = board, settings, clock
        self.conflict_check = conflict_check
        self.state = 'DISABLED'
        self.stop_requested = threading.Event()
        self._lock = threading.Lock()
        self._start_requested = False
        self._sensor_at = None
        self._sensor_valid = False
        self.last_now = None
        self.started = self.last_cycle = self.watchdog_started = None
        self.command_seq = 0
        self.segment = None
        self.watchdog_observation_started = None
        self.watchdog_zero_observed = False
        self.failed = False
        self.finished = False

    def now(self):
        value = finite(self.clock(), 'monotonic clock')
        if self.last_now is not None and value < self.last_now:
            raise OutputError('monotonic clock went backwards')
        self.last_now = value
        return value

    def transition(self, state, reason):
        self.state = state
        self.board.event('output_probe_state', state=state, reason=reason,
                         experiment=self.settings.experiment, segment=self.segment)

    def prepare(self, path, vehicle, context):
        self.conflict_check()
        if not self.board.trace.enabled or not self.board.trace.detailed:
            raise OutputError('full trace is required')
        self.transition('CONFIGURING', 'prepare')
        try:
            self.board.prepare(self.settings.limits, path, vehicle, context)
            self.transition('READY', 'configuration and signed roundtrip checked; not started')
        except Exception as error:
            self.board.event('output_configuration', success=False, error=str(error))
            self.fail(str(error))
            raise

    def sensor_received(self, at, valid=True):
        with self._lock:
            self._sensor_at, self._sensor_valid = at, valid

    def request_start(self):
        with self._lock:
            if self.state != 'READY' or self.stop_requested.is_set():
                return False, 'probe is not READY; restart after fault/restore'
            if self._start_requested:
                return False, 'start already requested'
            self._start_requested = True
        return True, 'finite experiment queued; I2C is handled by the worker'

    def request_stop(self):
        self.stop_requested.set()

    def guard(self):
        now = self.now()
        if self.stop_requested.is_set():
            raise OutputError('stop requested')
        if self.started is not None:
            if now - self.started >= self.settings.max_duration:
                raise OutputError('maximum experiment duration exceeded')
            if self.watchdog_observation_started is None:
                if now - self.last_cycle > self.settings.timeout:
                    raise OutputError('output command deadline exceeded')
                if (self.watchdog_started is not None and
                        now - self.watchdog_started >= self.settings.watchdog_ms * .001):
                    raise OutputError('watchdog lease exhausted')
            with self._lock:
                sensor_at, valid = self._sensor_at, self._sensor_valid
            if (not valid or sensor_at is None or now < sensor_at or
                    now - sensor_at > self.settings.sensor_timeout):
                raise OutputError('wheel odometry is absent, invalid or stale')
        trace = self.board.trace
        if not trace.enabled or not trace.detailed:
            raise OutputError('full trace unavailable')
        if any(getattr(trace, name, 0) for name in
               ('capture_errors', 'publish_errors', 'dropped')):
            raise OutputError('probe trace reported errors or dropped records')
        return now

    def tick(self):
        if self.finished:
            return
        if self.stop_requested.is_set():
            self.finish('stop requested')
            return
        with self._lock:
            start = self._start_requested
            self._start_requested = False
        try:
            if self.state == 'READY' and start:
                self.conflict_check()
                self.started = self.last_cycle = self.now()
                self.guard()
                self.transition('RUNNING', 'explicit start')
            if self.state != 'RUNNING':
                return
            self.conflict_check()
            now = self.guard()
            if self.watchdog_observation_started is not None:
                observation = self.board.readback(self.guard)
                expired = self.now() - self.watchdog_started >= self.settings.watchdog_ms * .001
                if expired and observation['m_out_signed'] == [0, 0]:
                    self.watchdog_zero_observed = True
                self.board.event('output_readback', command_seq=self.command_seq,
                                 watchdog_observation=True, watchdog_expired=expired,
                                 **observation)
                if self.now() - self.watchdog_observation_started >= self.settings.watchdog_wait:
                    self.board.event('output_watchdog_result',
                                     computed_zero_observed=self.watchdog_zero_observed,
                                     physical_stop_verified=False)
                    if not self.watchdog_zero_observed:
                        raise OutputError('no zero M_OUT observed after watchdog expiry')
                    self.finish('watchdog observation complete; inspect wheel log')
                return
            if now - self.started >= sum(r['duration_s'] for r in self.settings.sequence):
                if self.settings.experiment == 'watchdog':
                    self.watchdog_observation_started = now
                    self.board.event('output_watchdog_wait', command_seq=self.command_seq,
                                     last_watchdog_monotonic_s=self.watchdog_started)
                else:
                    self.finish('finite sequence complete')
                return
            if self.command_seq and now - self.last_cycle < self.settings.period:
                return
            # Verify the previous cycle deadline before assigning a fresh lease.
            self.guard()
            scheduled = (self.started if not self.command_seq else
                         self.last_cycle + self.settings.period)
            self.last_cycle = self.now()
            elapsed, index = now - self.started, 0
            for index, row in enumerate(self.settings.sequence):
                if elapsed < row['duration_s']:
                    break
                elapsed -= row['duration_s']
            changed = index != self.segment
            self.segment = index
            row = self.settings.sequence[index]
            self.command_seq += 1
            completed = False
            target = (row['target_offset_counts'] if changed and
                      self.settings.experiment == 'position' else None)
            try:
                self.watchdog_started = self.board.apply(
                    row['t_out'], self.settings.watchdog_ms, self.guard, target)
                completed = True
            finally:
                self.board.event('output_command', command_seq=self.command_seq,
                                 segment=index, t_out_requested=list(row['t_out']),
                                 t_out_attempted=list(row['t_out']),
                                 target_offset_counts=target,
                                 scheduled_monotonic_s=scheduled,
                                 cycle_started_monotonic_s=self.last_cycle,
                                 watchdog_started_monotonic_s=(self.watchdog_started
                                                              if completed else None),
                                 update_completed=completed)
            observation = self.board.readback(self.guard)
            self.board.event('output_readback', command_seq=self.command_seq,
                             watchdog_observation=False, **observation)
            if observation['output_enable'] != 3:
                raise OutputError('output unexpectedly disabled')
            if observation['t_out_signed'] != list(row['t_out']):
                raise OutputError('T_OUT changed after Enable')
            if any(abs(v) > cap for v, cap in
                   zip(observation['m_out_signed'], self.settings.limits)):
                raise OutputError('M_OUT exceeds the configured probe limit')
        except Exception as error:
            self.fail(str(error))

    def fail(self, reason):
        self.failed = True
        self.stop_requested.set()
        self.transition('FAULT', reason)
        self.finish(reason)

    def finish(self, reason):
        if self.finished:
            return
        self.stop_requested.set()
        self.transition('RESTORING', reason)
        errors = self.board.restore() if self.board.touched else []
        self.failed = self.failed or bool(errors)
        self.finished = True
        self.transition('FAULT' if self.failed else 'DISABLED',
                        'restoration incomplete' if errors else reason)


class ProbeLock(object):
    """Host-wide cooperative lock; never remove the inode after releasing it."""
    def __init__(self, path):
        import fcntl
        flags = os.O_RDWR | os.O_CREAT | getattr(os, 'O_NOFOLLOW', 0)
        descriptor = os.open(path, flags, 0o600)
        self.handle = os.fdopen(descriptor, 'a+')
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            self.handle.close()
            raise OutputError('another output probe holds the lock')

    def close(self):
        self.handle.close()


class ConflictMonitor(object):
    """ROS master traffic must not run in the periodic output worker."""
    def __init__(self, inspect_graph, clock):
        self.inspect_graph, self.clock = inspect_graph, clock
        self.error, self.checked_at = None, None
        self.stopping = threading.Event()
        self.poll()
        self.check()
        self.thread = threading.Thread(target=self.run, name='output-probe-graph')
        self.thread.daemon = True
        self.thread.start()

    def poll(self):
        try:
            self.inspect_graph()
            self.checked_at = self.clock()
        except Exception as error:
            # Latch failure; never resume an experiment after a graph fault.
            self.error = str(error)

    def check(self):
        if self.error is not None:
            raise OutputError(self.error)
        now = self.clock()
        if self.checked_at is None or not 0 <= now - self.checked_at <= 2.:
            raise OutputError('ROS graph conflict monitor is stale')

    def run(self):
        while not self.stopping.wait(.5):
            self.poll()

    def close(self):
        self.stopping.set()
        self.thread.join(.2)


def source_metadata():
    result = {}
    for name in ('motor_output.py', 'output_probe.py'):
        with open(os.path.join(os.path.dirname(__file__), name), 'rb') as handle:
            result[name] = hashlib.sha256(handle.read()).hexdigest()
    return result


def main():
    import rospy
    import rosgraph
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger, TriggerResponse
    from lightrover_ros.srv import Wrc201Msg
    from rover_drive_trace.trace import (boot_id, create_service_proxy,
                                        create_trace, make_monotonic)

    rospy.init_node('rover_output_probe')
    params = rospy.get_param('~', {})
    vehicle = rospy.get_namespace().strip('/')
    if not vehicle:
        raise OutputError('a vehicle namespace is required')
    restore_path = params.get('restore_snapshot', '')
    settings = None if restore_path else ProbeSettings(params)
    if params.get('trace_mode', 'full') != 'full':
        raise OutputError('output probe requires trace_mode=full')
    clock = make_monotonic()
    trace = create_trace(rospy, 'controller', dict(
        controller_model='output_probe', output_schema_version=1,
        wheel_order=['motor0', 'motor1'], input_units='T_OUT register counts; scaling unverified',
        additional_i2c_reads=True, probe_source_sha256=source_metadata(),
        experiment_settings=None if settings is None else settings.metadata(),
        firmware_semantics_verified=False))
    if not trace.enabled or not trace.detailed:
        raise OutputError('full trace initialization failed; no board access')
    master = rosgraph.Master(rospy.get_name())

    def conflict_check():
        publishers, subscribers, services = master.getSystemState()
        nodes = set(node for _, owners in publishers + subscribers + services for node in owners)
        conflicts = [node for node in nodes if node != rospy.get_name() and
                     (node.rsplit('/', 1)[-1].startswith(('pos_controller', 'rover_pos')))]
        # A velocity controller with a different node name is still a conflict.
        drive_topic = rospy.resolve_name('rover_drive')
        conflicts += [node for topic, owners in subscribers if topic == drive_topic
                      for node in owners if node != rospy.get_name()]
        if conflicts:
            raise OutputError('stop velocity controllers first: %s' % sorted(set(conflicts)))

    lock = ProbeLock('/tmp/harada-tsubakino-motor-output.lock')
    service = board = probe = monitor = None
    try:
        conflict_check()
        rospy.wait_for_service('wrc201_i2c', timeout=10)
        service = create_service_proxy(rospy, 'wrc201_i2c', Wrc201Msg,
                                       lambda: trace, persistent=True)
        board = MotorOutput(service, trace)
        if restore_path:
            snapshot = load_snapshot(os.path.expanduser(restore_path), vehicle)
            board.use_snapshot(snapshot)
            if board.restore():
                raise OutputError('restoration incomplete; inspect output_restore')
            rospy.loginfo('Restored with output disabled; no experiment was started.')
            return
        monitor = ConflictMonitor(conflict_check, clock)
        probe = OutputProbe(board, settings, clock, monitor.check)
        rospy.on_shutdown(probe.request_stop)

        def odometry(message):
            try:
                finite(message.twist.twist.linear.x, 'wheel velocity')
                finite(message.twist.twist.angular.z, 'wheel angular velocity')
                age = (rospy.Time.now() - message.header.stamp).to_sec()
                probe.sensor_received(clock(), 0 <= age <= settings.sensor_timeout)
            except Exception:
                probe.sensor_received(clock(), False)

        subscriber = rospy.Subscriber('odometry/wheel', Odometry, odometry,
                                      queue_size=1, tcp_nodelay=True)
        snapshot_path = os.path.abspath(os.path.expanduser(params.get(
            'snapshot_path', '~/.ros/motor-output-%s.json' % vehicle.replace('/', '-'))))
        if not os.path.isdir(os.path.dirname(snapshot_path)):
            os.makedirs(os.path.dirname(snapshot_path))
        probe.prepare(snapshot_path, vehicle, dict(boot_id=boot_id(),
                      run_id=trace.run_id, captured_monotonic_s=clock()))

        def start(_):
            accepted, message = probe.request_start()
            return TriggerResponse(success=accepted, message=message)

        def stop(_):
            probe.request_stop()
            return TriggerResponse(success=True, message='stop queued to I2C worker')

        start_service = rospy.Service('~start', Trigger, start)
        stop_service = rospy.Service('~stop', Trigger, stop)
        rospy.loginfo('READY, output disabled. Explicit ~start runs the finite lifted-wheel experiment.')
        while not rospy.is_shutdown() and not probe.finished:
            probe.tick()
            # Wall wait; /use_sim_time must not freeze the output watchdog worker.
            probe.stop_requested.wait(min(settings.period / 4, .02))
        if probe.failed:
            raise OutputError('probe stopped in FAULT; inspect trace and recovery snapshot')
    finally:
        try:
            if probe is not None:
                probe.finish('node shutdown')
        finally:
            try:
                if monitor is not None:
                    monitor.close()
                trace.close()
                if trace.detailed:
                    trace.flush()
            finally:
                try:
                    if service is not None:
                        service.close()
                finally:
                    lock.close()

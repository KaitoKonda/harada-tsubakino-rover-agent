"""Full events or constant-space timing summaries, published off the control path.

Python 2.7 (ROS Melodic) and Python 3 compatible. No hardware I/O here.
"""
import collections
import hashlib
import json
import math
import os
import threading
import time
import uuid


def make_monotonic():
    if hasattr(time, 'monotonic'):
        return time.monotonic
    # Python 2.7 on Raspberry Pi OS: use the same kernel clock as Python 3.
    import ctypes

    class Timespec(ctypes.Structure):
        _fields_ = [('tv_sec', ctypes.c_long), ('tv_nsec', ctypes.c_long)]

    library = ctypes.CDLL('librt.so.1', use_errno=True)
    clock_gettime = library.clock_gettime
    clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
    clock_gettime.restype = ctypes.c_int

    def monotonic():
        value = Timespec()
        if clock_gettime(1, ctypes.byref(value)):
            raise OSError(ctypes.get_errno(), 'clock_gettime(CLOCK_MONOTONIC)')
        return value.tv_sec + value.tv_nsec * 1e-9

    monotonic()
    return monotonic


class NullTrace(object):
    enabled = False
    detailed = False
    mode = 'off'

    def now(self):
        return None

    def emit(self, event, fields=None, at=None):
        pass

    def close(self):
        pass

    def observe(self, metric, value, failed=False):
        pass

    def count(self, name):
        pass


def resolve_mode(mode='', legacy_enabled=True):
    """An explicit mode takes precedence; empty means legacy true/false."""
    if mode is not None and mode != '':
        if mode not in ('full', 'minimal', 'off'):
            raise ValueError('trace_mode must be full, minimal, or off')
        return mode
    if legacy_enabled is True or legacy_enabled == 'true':
        return 'full'
    if legacy_enabled is False or legacy_enabled == 'false':
        return 'off'
    raise ValueError('trace_enabled must be true or false; use trace_mode for minimal')


class TracePublisher(object):
    enabled = True

    def __init__(self, rospy, stream, metadata, clock=None, start_worker=True,
                 mode='full'):
        from std_msgs.msg import String
        if mode not in ('full', 'minimal'):
            raise ValueError('TracePublisher requires full or minimal mode')
        self.mode = mode
        self.detailed = mode == 'full'
        self.rospy = rospy
        self.clock = clock or make_monotonic()
        self.message_type = String
        self.capacity = self.batch_size = self.publish_hz = 0
        if self.detailed:
            self.capacity = int(rospy.get_param('~trace_queue_capacity', 2048))
            self.batch_size = int(rospy.get_param('~trace_batch_size', 128))
            self.publish_hz = float(rospy.get_param('~trace_publish_hz', 20.0))
            if self.capacity < 1 or self.batch_size < 1 or not 0 < self.publish_hz <= 100:
                raise ValueError('invalid trace queue/batch/publish frequency')
        self.summary_interval = float(rospy.get_param('~trace_summary_interval', 5.0))
        if not 0 < self.summary_interval < float('inf'):
            raise ValueError('trace_summary_interval must be positive and finite')
        self.queue = collections.deque() if self.detailed else None
        self.lock = threading.Lock()
        self.timing_lock = threading.Lock()
        self.timing_metrics = {}
        self.timing_counters = {}
        self.timing_sequence = 0
        self.stop = threading.Event()
        self.sequence = 0
        self.batch_sequence = 0
        self.dropped = 0
        self.capture_errors = 0
        self.publish_errors = 0
        self.window_started = self.clock()
        self.run_id = uuid.uuid4().hex
        self.node = rospy.get_name()
        self.metadata = dict(metadata)
        self.metadata['trace_mode'] = self.mode
        self.metadata['summary_interval_s'] = self.summary_interval
        self.metadata['clock'] = 'CLOCK_MONOTONIC seconds, shared within one Pi boot'
        self.metadata['source_sha256'] = source_hashes()
        self.metadata['boot_id'] = boot_id()
        self.publisher = (rospy.Publisher('drive_trace/' + stream, String,
                                          queue_size=10) if self.detailed else None)
        self.timing_publisher = rospy.Publisher(
            'drive_trace/' + stream + '_timing', String, queue_size=2)
        self.metadata_publisher = rospy.Publisher(
            'drive_trace/' + stream + '_metadata', String, queue_size=1, latch=True)
        self.thread = None
        if start_worker:
            self.thread = threading.Thread(target=self._run, name='drive-trace')
            self.thread.daemon = True
            self.thread.start()
        rospy.on_shutdown(self.close)

    def now(self):
        try:
            return self.clock()
        except Exception:
            self.capture_errors += 1
            return None

    def emit(self, event, fields=None, at=None):
        if not self.detailed:
            return
        # Callers own/freeze fields before enqueuing; never enqueue live lists.
        try:
            record = dict(fields or {})
            record.update(event=event, monotonic_s=self.now() if at is None else at,
                          ros_time_ns=self.rospy.Time.now().to_nsec())
            with self.lock:
                self.sequence += 1
                record['seq'] = self.sequence
                if len(self.queue) >= self.capacity:
                    self.queue.popleft()
                    self.dropped += 1
                self.queue.append(record)
        except Exception:
            self.capture_errors += 1

    def flush(self):
        if not self.detailed:
            return
        with self.lock:
            events = [self.queue.popleft()
                      for _ in range(min(self.batch_size, len(self.queue)))]
            stats = dict(last_event_seq=self.sequence, queue_dropped=self.dropped,
                         capture_errors=self.capture_errors, queued=len(self.queue))
        self.batch_sequence += 1
        envelope = dict(schema_version=1, run_id=self.run_id, node=self.node,
                        batch_seq=self.batch_sequence, events=events,
                        publish_errors=self.publish_errors, **stats)
        try:
            payload = json.dumps(envelope, separators=(',', ':'), allow_nan=False)
            self.publisher.publish(self.message_type(data=payload))
        except Exception:
            self.publish_errors += 1

    def observe(self, metric, value, failed=False):
        """Constant-space aggregation, identical in full and minimal modes."""
        if value is None:
            return
        try:
            if math.isnan(value) or math.isinf(value):
                raise ValueError('non-finite timing')
            with self.timing_lock:
                sample = self.timing_metrics.get(metric)
                if sample is None:
                    # count, sum, min, max, first, last, failures
                    self.timing_metrics[metric] = [1, value, value, value,
                                                   value, value, int(failed)]
                else:
                    sample[0] += 1
                    sample[1] += value
                    sample[2] = min(sample[2], value)
                    sample[3] = max(sample[3], value)
                    sample[5] = value
                    sample[6] += int(failed)
        except Exception:
            self.capture_errors += 1

    def count(self, name):
        with self.timing_lock:
            self.timing_counters[name] = self.timing_counters.get(name, 0) + 1

    def flush_timing(self, partial=False):
        # Swap under the lock; formatting/JSON/network work happens afterward.
        with self.timing_lock:
            ended = self.now()
            if ended is None:
                return
            metrics, self.timing_metrics = self.timing_metrics, {}
            counters, self.timing_counters = self.timing_counters, {}
            started, self.window_started = self.window_started, ended
        self.timing_sequence += 1
        formatted = {}
        for name, sample in metrics.items():
            formatted[name] = dict(count=sample[0], sum=sample[1],
                mean=sample[1]/float(sample[0]), min=sample[2], max=sample[3],
                first=sample[4], last=sample[5], failures=sample[6])
        envelope = dict(schema_version=1, run_id=self.run_id, node=self.node,
            trace_mode=self.mode, summary_seq=self.timing_sequence,
            window_start_monotonic_s=started, window_end_monotonic_s=ended,
            window_duration_s=ended-started, partial=partial,
            metrics=formatted, counters=counters,
            capture_errors=self.capture_errors, publish_errors=self.publish_errors)
        try:
            self.timing_publisher.publish(self.message_type(data=json.dumps(
                envelope, separators=(',', ':'), allow_nan=False)))
        except Exception:
            self.publish_errors += 1

    def publish_metadata(self):
        envelope = dict(schema_version=1, run_id=self.run_id, node=self.node,
                        metadata=self.metadata)
        try:
            self.metadata_publisher.publish(self.message_type(data=json.dumps(
                envelope, separators=(',', ':'), allow_nan=False)))
        except Exception:
            self.publish_errors += 1

    def _run(self):
        last_metadata = -float('inf')
        last_summary = self.window_started
        while not self.stop.is_set():
            now = self.now()
            metadata_interval = 1.0 if self.detailed else self.summary_interval
            if now is not None and now - last_metadata >= metadata_interval:
                self.publish_metadata()
                last_metadata = now
            if now is not None and now - last_summary >= self.summary_interval:
                self.flush_timing()
                last_summary = self.window_started
            if self.detailed:
                self.flush()
            self.stop.wait(1.0 / self.publish_hz if self.detailed else self.summary_interval)
        self.flush_timing(partial=True)

    def close(self):
        self.stop.set()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(0.25)
        # Shutdown is best effort; do not wait for network or drain forever.


def boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id') as handle:
            return handle.read().strip()
    except IOError:
        return None


def source_hashes():
    result = {}
    for name in ('controller.py', 'odometry.py', 'motor.py', 'trace.py'):
        with open(os.path.join(os.path.dirname(__file__), name), 'rb') as handle:
            result[name] = hashlib.sha256(handle.read()).hexdigest()
    return result


def create_trace(rospy, stream, metadata):
    mode = resolve_mode(rospy.get_param('~trace_mode', ''),
                        rospy.get_param('~trace_enabled', True))
    if mode == 'off':
        return NullTrace()
    try:
        return TracePublisher(rospy, stream, metadata, mode=mode)
    except Exception as error:
        # Telemetry failure must not disable the existing drive process.
        rospy.logwarn('Drive trace disabled: %s', error)
        return NullTrace()


def difference(end, start):
    return None if end is None or start is None else end - start


def timed_call(trace, call, args, records):
    """Keep service request order, return values, and exception behavior intact."""
    if not trace.enabled:
        return call(*args)
    start = trace.now()
    record = (dict(address=args[0], data=args[1], length=args[2], operation=args[3],
                   start_monotonic_s=start, success=False) if trace.detailed else None)
    success = False
    try:
        response = call(*args)
        success = True
        if trace.detailed:
            record['success'] = True
            record['response_read_data'] = response.readData
        return response
    except Exception as error:
        if trace.detailed:
            record['error_type'] = type(error).__name__
            record['error'] = str(error)
        raise
    finally:
        end = trace.now()
        duration = difference(end, start)
        trace.observe('service_%s_0x%02x_s' % (args[3], args[0]), duration,
                      failed=not success)
        if trace.detailed:
            record['end_monotonic_s'] = end
            record['duration_s'] = duration
            records.append(record)

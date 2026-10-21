"""Bounded producer queue; JSON encoding and ROS publishing run off the control path.

Python 2.7 (ROS Melodic) and Python 3 compatible. No hardware I/O here.
"""
import collections
import hashlib
import json
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

    def now(self):
        return None

    def emit(self, event, fields=None, at=None):
        pass

    def close(self):
        pass


class TracePublisher(object):
    enabled = True

    def __init__(self, rospy, stream, metadata, clock=None, start_worker=True):
        from std_msgs.msg import String
        self.rospy = rospy
        self.clock = clock or make_monotonic()
        self.message_type = String
        self.capacity = int(rospy.get_param('~trace_queue_capacity', 2048))
        self.batch_size = int(rospy.get_param('~trace_batch_size', 128))
        self.publish_hz = float(rospy.get_param('~trace_publish_hz', 20.0))
        if self.capacity < 1 or self.batch_size < 1 or not 0 < self.publish_hz <= 100:
            raise ValueError('invalid trace queue/batch/publish frequency')
        self.queue = collections.deque()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sequence = 0
        self.batch_sequence = 0
        self.dropped = 0
        self.capture_errors = 0
        self.publish_errors = 0
        self.run_id = uuid.uuid4().hex
        self.node = rospy.get_name()
        self.metadata = dict(metadata)
        self.metadata['clock'] = 'CLOCK_MONOTONIC seconds, shared within one Pi boot'
        self.metadata['source_sha256'] = source_hashes()
        self.metadata['boot_id'] = boot_id()
        self.publisher = rospy.Publisher('drive_trace/' + stream, String,
                                         queue_size=10)
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
        while not self.stop.is_set():
            now = self.now()
            if now is not None and now - last_metadata >= 1.0:
                self.publish_metadata()
                last_metadata = now
            self.flush()
            self.stop.wait(1.0 / self.publish_hz)

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
    if not rospy.get_param('~trace_enabled', True):
        return NullTrace()
    try:
        return TracePublisher(rospy, stream, metadata)
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
    record = dict(address=args[0], data=args[1], length=args[2], operation=args[3],
                  start_monotonic_s=start, success=False)
    try:
        response = call(*args)
        record['success'] = True
        record['response_read_data'] = response.readData
        return response
    except Exception as error:
        record['error_type'] = type(error).__name__
        record['error'] = str(error)
        raise
    finally:
        end = trace.now()
        record['end_monotonic_s'] = end
        record['duration_s'] = difference(end, start)
        records.append(record)

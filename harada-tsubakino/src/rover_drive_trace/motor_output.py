"""Experimental VS-WRC201 output access. Python 2.7 / 3, no ROS imports.

T_OUT semantics must be measured; this module does not assume it is a PWM API.
The vendor service writes multi-byte registers one byte at a time. Output must
be disabled before changing T_OUT, including during every running update.
"""
import json
import math
import os

from rover_drive_trace.trace import timed_call

ENABLE, WDT = 0x10, 0x08
GAIN, PUNCH, LIMIT = (0x20, 0x22), (0x30, 0x32), (0x2c, 0x2e)
TARGET, OFFSET, OUTPUT = (0x40, 0x44), (0x50, 0x52), (0x6c, 0x6e)
POSITION = (0x60, 0x64)
SAVED = tuple((address, 2) for pair in (GAIN, PUNCH, LIMIT, OFFSET)
              for address in pair) + tuple((address, 4) for address in TARGET)


class OutputError(Exception):
    pass


def integer(value, lower, upper, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        # Python 2 long values are also accepted without truncating floats.
        try:
            valid_long = isinstance(value, long) and not isinstance(value, bool)
        except NameError:
            valid_long = False
        if not valid_long:
            raise ValueError('%s must be an integer' % name)
    if value != int(value) or not lower <= value <= upper:
        raise ValueError('%s must be in [%s, %s]' % (name, lower, upper))
    return int(value)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('%s must be finite' % name)
    result = float(value)
    if math.isnan(result) or math.isinf(result):
        raise ValueError('%s must be finite' % name)
    return result


def signed(value, bits):
    raw = int(value) & ((1 << bits) - 1)
    return raw - (1 << bits) if raw & (1 << (bits - 1)) else raw


def save_snapshot(path, snapshot):
    """Exclusive, durable file: never overwrite a previous recovery record."""
    encoded = json.dumps(snapshot, sort_keys=True, allow_nan=False).encode('utf-8')
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, 'wb') as handle:
        handle.write(encoded)
        handle.flush()
        os.fsync(handle.fileno())
    # Persist the directory entry as well on the target Linux filesystem.
    if hasattr(os, 'O_DIRECTORY'):
        descriptor = os.open(os.path.dirname(os.path.abspath(path)), os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def load_snapshot(path, vehicle):
    with open(path, 'r') as handle:
        value = json.load(handle)
    if (value.get('schema_version') != 1 or value.get('kind') != 'motor_output_snapshot'
            or value.get('vehicle') != vehicle):
        raise OutputError('snapshot schema/vehicle mismatch')
    registers = value.get('registers', {})
    if set(registers) != set(str(address) for address, _ in SAVED):
        raise OutputError('snapshot register set mismatch')
    for address, length in SAVED:
        integer(registers[str(address)], 0, (1 << (8 * length)) - 1, 'snapshot register')
    for key in ('system_name', 'firmware_revision'):
        integer(value.get(key), 0, 65535, key)
    return value


class MotorOutput(object):
    def __init__(self, service, trace):
        self.service, self.trace = service, trace
        self.records = []
        self.snapshot = None
        self.touched = False
        self.configured = False

    def read(self, address, length=2):
        response = timed_call(self.trace, self.service,
                              (address, 0, length, 'r'), self.records)
        return int(response.readData) & ((1 << (8 * length)) - 1)

    def write(self, address, value, length=2):
        integer(value, -(1 << (8 * length - 1)), (1 << (8 * length)) - 1, 'register')
        self.touched = True
        response = timed_call(self.trace, self.service,
                              (address, int(value), length, 'w'), self.records)
        if response.readData != 1:
            raise OutputError('write not acknowledged: 0x%02x' % address)

    def checked(self, address, value, length=2):
        self.write(address, value, length)
        observed = self.read(address, length)
        if observed != (int(value) & ((1 << (8 * length)) - 1)):
            raise OutputError('readback mismatch: 0x%02x' % address)

    def event(self, name, **fields):
        records, self.records = self.records, []
        self.trace.emit(name, dict(fields, service_calls=records))

    def disable(self):
        self.checked(ENABLE, 0, 1)

    def prepare(self, limits, path, vehicle, context):
        self.configured = False
        if not isinstance(limits, (list, tuple)) or len(limits) != 2:
            raise ValueError('two output limits are required')
        self.limits = tuple(integer(v, 1, 4096, 'output limit') for v in limits)
        self.disable()
        self.snapshot = dict(context, schema_version=1, kind='motor_output_snapshot',
                             vehicle=vehicle, system_name=self.read(0x00),
                             firmware_revision=self.read(0x02),
                             watchdog_before=self.read(WDT),
                             registers=dict((str(a), self.read(a, n)) for a, n in SAVED))
        # No gain/offset/limit changes until the complete recovery file is durable.
        save_snapshot(path, self.snapshot)
        for address in OFFSET:
            self.checked(address, 0)
        for pair in (GAIN, PUNCH):
            for address in pair:
                self.checked(address, 0)
        for address, limit in zip(LIMIT, limits):
            self.checked(address, limit)
        self.checked(WDT, 0)
        # Test signed transport with motors disabled; never restore a live offset.
        for address in OFFSET:
            for value in (-1, 1, 0):
                self.checked(address, value)
        self.configured = True
        self.event('output_configuration', snapshot=self.snapshot,
                   snapshot_path=path, output_limits=list(limits), success=True)

    def verify_configuration(self, limits, guard):
        for pair in (GAIN, PUNCH):
            for address in pair:
                guard()
                if self.read(address) != 0:
                    raise OutputError('gain/punch changed: 0x%02x' % address)
        for address, limit in zip(LIMIT, limits):
            guard()
            if self.read(address) != limit:
                raise OutputError('output limit changed: 0x%02x' % address)

    def apply(self, values, watchdog_ms, guard, target_offset=None):
        """One serialized update; guard is checked before *every* bus request.

        A rejected/stalled write must never be followed by Enable or WDT refresh.
        """
        if not self.configured:
            raise OutputError('output configuration has not completed')
        if not isinstance(values, (list, tuple)) or len(values) != 2:
            raise ValueError('two output values are required')
        values = tuple(integer(v, -cap, cap, 'T_OUT')
                       for v, cap in zip(values, self.limits))
        watchdog_ms = integer(watchdog_ms, 1, 65534, 'watchdog_ms')
        if target_offset is not None:
            if not isinstance(target_offset, (list, tuple)) or len(target_offset) != 2:
                raise ValueError('two target offsets are required')
            target_offset = tuple(integer(v, -4096, 4096, 'target offset')
                                  for v in target_offset)
        def checked(address, value, length=2):
            guard()
            self.write(address, value, length)
            guard()
            if self.read(address, length) != (value & ((1 << (8 * length)) - 1)):
                raise OutputError('output/configuration readback mismatch')

        checked(ENABLE, 0, 1)
        self.verify_configuration(self.limits, guard)
        if target_offset is not None:
            for address, measured, delta in zip(TARGET, POSITION, target_offset):
                guard()
                position = self.read(measured, 4)
                checked(address, signed(position + delta, 32), 4)
        for address, value in zip(OFFSET, values):
            checked(address, value)
        guard()
        watchdog_started = guard()
        self.write(WDT, watchdog_ms)
        guard()
        # Do not re-enable if writing the timer already consumed its whole lease.
        if guard() - watchdog_started >= watchdog_ms * 0.001:
            raise OutputError('watchdog expired before Enable')
        self.write(ENABLE, 3, 1)
        guard()
        if self.read(ENABLE, 1) != 3:
            raise OutputError('Enable not confirmed')
        if guard() - watchdog_started >= watchdog_ms * 0.001:
            raise OutputError('watchdog lease exhausted during update')
        return watchdog_started

    def readback(self, guard):
        def read(address, length=2):
            guard()
            return self.read(address, length)
        offsets = [read(address) for address in OFFSET]
        outputs = [read(address) for address in OUTPUT]
        enabled = read(ENABLE, 1)
        return dict(t_out_raw=offsets, t_out_signed=[signed(v, 16) for v in offsets],
                    m_out_raw=outputs, m_out_signed=[signed(v, 16) for v in outputs],
                    output_enable=enabled)

    def restore(self):
        """Try independent stop operations; restore gains only after proven disable.

        Keep WDT armed if output disable/zeroing cannot be confirmed. No request
        that might have succeeded is blindly retried; report unresolved items.
        """
        self.configured = False
        errors = []

        def attempt(address, value, length=2):
            try:
                self.checked(address, value, length)
                return True
            except Exception as error:
                errors.append(dict(address=address, error=str(error)))
                return False

        disabled = attempt(ENABLE, 0, 1)
        zeroed = [attempt(address, 0) for address in OFFSET]
        if disabled and all(zeroed):
            if self.snapshot is not None:
                for address, length in SAVED:
                    if address not in OFFSET:
                        attempt(address, self.snapshot['registers'][str(address)], length)
            attempt(WDT, 0)
        self.event('output_restore', success=not errors, output_disabled=disabled,
                   offsets_zeroed=zeroed, unresolved=errors)
        return errors

    def use_snapshot(self, snapshot):
        # Identity is checked before applying saved gains/targets. No arbitrary
        # addresses from a file are ever used as write destinations.
        if (self.read(0x00) != snapshot['system_name'] or
                self.read(0x02) != snapshot['firmware_revision']):
            raise OutputError('board identity/revision differs from snapshot')
        self.snapshot = snapshot

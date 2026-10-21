"""Exercise serial framing without ROS, hardware, or third-party dependencies."""
import os
import runpy
import sys
import types
import unittest
import threading
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "harada-tsubakino", "src"))


class SerialError(Exception):
    pass


class Message(object):
    def __init__(self):
        self.header = types.ModuleType("header")
        self.vector = types.ModuleType("vector")


class Publisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class SerialInput(object):
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.readSizes = []

    def readline(self, size):
        self.readSizes.append(size)
        chunk = self.chunks.pop(0)
        if isinstance(chunk, Exception):
            raise chunk
        if len(chunk) > size:
            self.chunks.insert(0, chunk[size:])
            return chunk[:size]
        return chunk

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True


class SerialSensorBridgeTests(unittest.TestCase):
    def setUp(self):
        self.warnings = []
        self.errors = []
        self.infos = []
        self.time = 0.0
        rospy = types.ModuleType("rospy")
        rospy.Time = type("Time", (), {"now": staticmethod(lambda: self.time)})
        rospy.logwarn_throttle = lambda interval, *args: self.warnings.append(args)
        rospy.logerr_throttle = lambda interval, *args: self.errors.append(args)
        rospy.logwarn = lambda *args: self.warnings.append(args)
        rospy.logerr = lambda *args: self.errors.append(args)
        rospy.loginfo = lambda *args: self.infos.append(args)
        rospy.loginfo_throttle = lambda interval, *args: self.infos.append(args)
        rospy.sleep = lambda duration: None
        serial = types.ModuleType("serial")
        serial.SerialException = SerialError
        tf = types.ModuleType("tf")
        tf.transformations = types.ModuleType("transformations")
        tf.transformations.quaternion_from_euler = lambda *args: args
        geometry = types.ModuleType("geometry_msgs")
        messages = types.ModuleType("geometry_msgs.msg")
        messages.Pose2D = Message
        messages.Vector3Stamped = Message
        geometry.msg = messages
        modules = {"rospy": rospy, "serial": serial, "tf": tf,
                   "geometry_msgs": geometry, "geometry_msgs.msg": messages}
        self.originalModules = dict((name, sys.modules.get(name)) for name in modules)
        sys.modules.update(modules)
        self.addCleanup(self.restoreModules)
        path = os.path.join(os.path.dirname(__file__), "..", "harada-tsubakino",
                            "scripts", "serialSensorBridge.py")
        bridgeClass = runpy.run_path(path)["SerialSensorBridge"]
        self.bridgeClass = bridgeClass
        self.bridge = bridgeClass.__new__(bridgeClass)
        self.bridge.port = "/dev/ttyACM0"
        self.bridge.receiveTimeout = 1.0
        self.bridge.startupGrace = 5.0
        self.bridge.warningInterval = 5.0
        self.bridge.initializeReceiveMonitor(lambda: self.time)
        self.bridge.serialBuffer = b""
        self.bridge.discardSerialLine = False
        self.bridge.pingInterval = 0.3
        self.bridge.lastPingTime = 0.0
        self.bridge.posePublisher = Publisher()
        self.bridge.rpyPublisher = Publisher()
        self.bridge.accelPublisher = Publisher()
        self.bridge.otosBaseLinkFrame = "pi1/otos_base_link"
        self.bridge.otosOdomFrame = "pi1/otos_odom"
        self.bridge.imuFrame = "pi1/hmc6343_link"
        self.bridge.tfBroadcaster = types.ModuleType("broadcaster")
        self.bridge.tfBroadcaster.sendTransform = lambda *args: None
        self.rospy = rospy

    def restoreModules(self):
        for name, original in self.originalModules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original

    def runChunks(self, chunks):
        self.bridge.serial = SerialInput(chunks)

        def isShutdown():
            self.time += 0.2
            return not self.bridge.serial.chunks

        self.rospy.is_shutdown = isShutdown
        self.bridge.run()

    def test_reported_partial_record_survives_empty_timeout(self):
        self.runChunks([b"OTOS,3408057,0.345459,-6.8", b"",
                        b"12345,0.125000\r\n"])
        poses = self.bridge.posePublisher.messages
        self.assertEqual(len(poses), 1)
        self.assertAlmostEqual(poses[0].y, -6.812345)
        self.assertAlmostEqual(poses[0].theta, 0.125)
        self.assertEqual(self.warnings, [])
        self.assertIn(b"PING\n", self.bridge.serial.writes)
        self.assertTrue(all(size == 257 for size in self.bridge.serial.readSizes))

    def test_split_final_number_is_not_published_until_newline(self):
        self.bridge.handleSerialChunk(b"OTOS,1,0.1,0.2,0.12")
        self.assertEqual(self.bridge.posePublisher.messages, [])
        self.bridge.handleSerialChunk(b"3456\n")
        self.assertAlmostEqual(self.bridge.posePublisher.messages[0].theta, 0.123456)

    def test_malformed_complete_record_does_not_block_next_record(self):
        self.runChunks([b"OTOS,1,0.1,-6.8\n", b"OTOS,2,0.1,0.2,0.3\n"])
        self.assertEqual(len(self.warnings), 1)
        self.assertEqual(len(self.bridge.posePublisher.messages), 1)

    def test_overlong_record_discards_remainder_until_newline(self):
        self.runChunks([b"X" * 300, b"OTOS,1,9,9,9\n",
                        b"OTOS,2,0.1,0.2,0.3\n"])
        self.assertEqual(len(self.warnings), 1)
        poses = self.bridge.posePublisher.messages
        self.assertEqual(len(poses), 1)
        self.assertEqual(poses[0].x, 0.1)
        self.assertEqual(self.bridge.serialBuffer, b"")

    def test_overlong_complete_record_does_not_discard_next_record(self):
        self.runChunks([b"X" * 256 + b"\n", b"OTOS,2,0.1,0.2,0.3\n"])
        self.assertEqual(len(self.warnings), 1)
        self.assertEqual(len(self.bridge.posePublisher.messages), 1)

    def test_read_error_resynchronizes_at_newline(self):
        self.runChunks([b"OTOS,1,0.", SerialError("USB read failed"),
                        b"OTOS,2,9,9,9\n", b"OTOS,3,0.1,0.2,0.3\n"])
        self.assertEqual(len(self.errors), 1)
        poses = self.bridge.posePublisher.messages
        self.assertEqual(len(poses), 1)
        self.assertEqual(poses[0].x, 0.1)

    def test_split_hmc_record_and_blank_lines(self):
        self.runChunks([b"\r\n", b"HMC,1,0.1,0.2,", b"0.3,1,2,3\n"])
        self.assertEqual(len(self.bridge.rpyPublisher.messages), 1)
        self.assertEqual(self.bridge.accelPublisher.messages[0].vector.z, 3.0)
        self.assertEqual(self.warnings, [])

    def sendHealthyRecords(self):
        self.bridge.handleSerialChunk(b"OTOS,1,0.1,0.2,0.3\n")
        self.bridge.handleSerialChunk(b"HMC,1,0.1,0.2,0.3,1,2,3\n")

    def test_silent_start_reports_timeout_after_grace_and_repeats_sparingly(self):
        self.time = 4.9
        self.bridge.checkReceiveHealth()
        self.assertEqual(self.warnings, [])
        self.time = 5.0
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.warnings), 1)
        self.assertIn("Serial receive timeout", self.warnings[0][0])
        self.time = 9.9
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.warnings), 1)
        self.time = 10.0
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.warnings), 2)

    def test_silence_after_healthy_stream_is_detected_and_recovery_is_logged(self):
        self.sendHealthyRecords()
        self.bridge.checkReceiveHealth()
        self.assertIn("stream healthy", self.infos[-1][0])
        self.time = 1.1
        self.bridge.checkReceiveHealth()
        self.assertIn("Serial receive timeout", self.warnings[-1][0])
        self.time = 1.2
        self.sendHealthyRecords()
        self.bridge.checkReceiveHealth()
        self.assertIn("stream healthy", self.infos[-1][0])
        self.assertEqual(self.bridge.receiveWarnings, {})

    def test_status_bytes_do_not_hide_missing_sensor_records(self):
        self.time = 5.1
        self.bridge.handleSerialChunk(b"STATUS,NO_SENSORS_READY\n")
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.errors), 1)
        self.assertEqual(len(self.warnings), 2)
        self.assertEqual({w[1] for w in self.warnings}, {"OTOS", "HMC"})
        self.assertTrue(all(w[-1] == "NO_SENSORS_READY" for w in self.warnings))

    def test_missing_sensor_data_followed_by_silence_does_not_claim_recovery(self):
        self.time = 5.1
        self.bridge.handleSerialChunk(b"STATUS,NO_SENSORS_READY\n")
        self.bridge.checkReceiveHealth()
        infoCount = len(self.infos)
        self.time = 6.2
        self.bridge.checkReceiveHealth()
        self.assertIn("Serial receive timeout", self.warnings[-1][0])
        self.assertEqual(len(self.infos), infoCount)

    def test_hmc_continues_but_otos_stops(self):
        self.sendHealthyRecords()
        self.bridge.checkReceiveHealth()
        self.time = 1.1
        self.bridge.handleSerialChunk(b"HMC,2,0.1,0.2,0.3,1,2,3\n")
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.warnings), 1)
        self.assertEqual(self.warnings[0][1], "OTOS")
        self.time = 1.2
        self.bridge.handleSerialChunk(b"OTOS,2,0.1,0.2,0.3\n")
        self.bridge.checkReceiveHealth()
        self.assertIn("stream healthy", self.infos[-1][0])

    def test_unterminated_bytes_do_not_count_as_valid_sensor_data(self):
        self.time = 5.1
        self.bridge.handleSerialChunk(b"OTOS,1,0.1,0.2,")
        self.bridge.checkReceiveHealth()
        self.assertEqual(len(self.warnings), 2)
        self.assertEqual(self.bridge.receivedCounts['OTOS'], 0)

    def test_nan_and_infinity_are_not_published_or_counted_as_healthy(self):
        for value in ("nan", "inf", "-inf"):
            self.bridge.handleSerialChunk(("OTOS,1,%s,0,0\n" % value).encode('ascii'))
            self.bridge.handleSerialChunk(("HMC,1,0,0,0,0,0,%s\n" % value).encode('ascii'))
        self.assertEqual(self.bridge.posePublisher.messages, [])
        self.assertEqual(self.bridge.rpyPublisher.messages, [])
        self.assertEqual(self.bridge.accelPublisher.messages, [])
        self.assertEqual(self.bridge.receivedCounts['invalid'], 6)
        self.assertEqual(self.bridge.receivedCounts['OTOS'], 0)
        self.assertEqual(self.bridge.receivedCounts['HMC'], 0)

    def test_arduino_failure_and_stream_off_are_prominent(self):
        for line in ("STATUS,OTOS_INIT_FAILED", "STATUS,HMC_INIT_FAILED", "ERROR,USB_FAULT"):
            self.bridge.handleSerialChunk((line + '\n').encode('ascii'))
        self.assertEqual(len(self.errors), 3)
        self.bridge.handleSerialChunk(b"STATUS,BRIDGE_TIMEOUT\n")
        self.assertIn("streaming warning", self.warnings[-1][0])
        self.assertEqual(self.warnings[-1][1], "BRIDGE_TIMEOUT")

    def test_receive_monitor_uses_its_clock_not_ros_time_and_no_serial_calls(self):
        self.rospy.Time.now = lambda: 0.0
        self.bridge.serial = None
        self.time = 6.0
        self.bridge.checkReceiveHealth()
        self.assertIn("Serial receive timeout", self.warnings[-1][0])

    def test_background_monitor_reports_without_running_serial_loop(self):
        reported = threading.Event()
        self.rospy.logwarn = lambda *args: reported.set()
        self.bridge.serial = None
        self.time = 6.0
        worker = threading.Thread(target=self.bridge.monitorReceive)
        worker.start()
        try:
            self.assertTrue(reported.wait(2.0))
        finally:
            self.bridge.monitorStop.set()
            worker.join(2.0)
        self.assertFalse(worker.is_alive())

    def test_shutdown_stops_monitor_before_sending_stop(self):
        self.bridge.serial = SerialInput([])
        self.bridge.onShutdown()
        self.assertTrue(self.bridge.monitorStop.is_set())
        self.assertEqual(self.bridge.serial.writes, [b"STOP\n"])
        self.assertTrue(self.bridge.serial.closed)

    def test_constructor_starts_independent_monitor_and_keeps_reset_start_sequence(self):
        self.rospy.get_param = lambda key, default: default
        self.rospy.Duration = lambda value: value
        self.rospy.Time.__new__ = staticmethod(lambda cls, *args: 0.0)
        self.rospy.Publisher = lambda *args, **kwargs: Publisher()
        self.rospy.on_shutdown = lambda callback: None
        sys.modules['tf'].TransformBroadcaster = lambda: self.bridge.tfBroadcaster
        serialInput = SerialInput([])
        sys.modules['serial'].Serial = lambda **kwargs: serialInput
        with patch('threading.Thread') as thread:
            bridge = self.bridgeClass()
        thread.return_value.start.assert_called_once_with()
        self.assertIs(thread.call_args.kwargs['target'].__self__, bridge)
        self.assertEqual(serialInput.writes, [b"RESET\n", b"START\n"])

    def test_serial_node_output_is_visible_and_unbuffered_in_launcher(self):
        launch = ET.parse(os.path.join(ROOT, 'harada-tsubakino', 'launch', 'harada-tsubakino.launch'))
        nodes = launch.findall(".//node[@name='serial_sensor_bridge']")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].get('output'), 'screen')
        self.assertEqual(nodes[0].find("env[@name='PYTHONUNBUFFERED']").get('value'), '1')


if __name__ == "__main__":
    unittest.main()

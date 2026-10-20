"""Exercise serial framing without ROS, hardware, or third-party dependencies."""
import os
import runpy
import sys
import types
import unittest


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


class SerialSensorBridgeTests(unittest.TestCase):
    def setUp(self):
        self.warnings = []
        self.errors = []
        self.time = 0.0
        rospy = types.ModuleType("rospy")
        rospy.Time = type("Time", (), {"now": staticmethod(lambda: self.time)})
        rospy.logwarn_throttle = lambda interval, *args: self.warnings.append(args)
        rospy.logerr_throttle = lambda interval, *args: self.errors.append(args)
        rospy.loginfo_throttle = lambda *args: None
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
        self.bridge = bridgeClass.__new__(bridgeClass)
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


if __name__ == "__main__":
    unittest.main()

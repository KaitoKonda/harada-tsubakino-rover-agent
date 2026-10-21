#!/usr/bin/env python2

import rospy
import serial
import tf
import math
import threading
from geometry_msgs.msg import Pose2D, Vector3Stamped
from rover_drive_trace.trace import make_monotonic


class SerialSensorBridge:
    # Sensor records are short ASCII lines; bound storage if a newline is lost.
    MAX_SERIAL_LINE_BYTES = 256

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        timeout = rospy.get_param("~timeout", 0.2)
        startupDelay = rospy.get_param("~startup_delay", 2.0)
        pingInterval = rospy.get_param("~ping_interval", 0.3)
        self.port = port
        self.receiveTimeout = float(rospy.get_param("~receive_timeout", 1.0))
        self.startupGrace = float(rospy.get_param("~receive_startup_grace", 5.0))
        self.warningInterval = float(rospy.get_param("~receive_warning_interval", 5.0))
        for value in (self.receiveTimeout, self.startupGrace, self.warningInterval):
            if not 0 < value < float("inf"):
                raise ValueError("serial receive monitor intervals must be positive and finite")

        self.otosOdomFrame = rospy.get_param("~otos_odom_frame", "otos_odom")
        self.otosBaseLinkFrame = rospy.get_param(
            "~otos_base_link_frame", "otos_base_link"
        )
        self.imuFrame = rospy.get_param("~imu_frame", "hmc6343_link")
        self.pingInterval = rospy.Duration(pingInterval)
        self.lastPingTime = rospy.Time(0)
        self.serialBuffer = b""
        self.discardSerialLine = False

        self.posePublisher = rospy.Publisher("odometry/otos", Pose2D, queue_size=20)
        self.rpyPublisher = rospy.Publisher("hmc6343_rpy", Vector3Stamped, queue_size=10)
        self.accelPublisher = rospy.Publisher("hmc6343_accel", Vector3Stamped, queue_size=10)

        self.tfBroadcaster = tf.TransformBroadcaster()
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.initializeReceiveMonitor(make_monotonic())
        rospy.on_shutdown(self.onShutdown)

        rospy.loginfo("Opened serial port %s at %d baud", port, baud)
        # Monitor independently: USB reads/writes may themselves stop returning.
        self.monitorThread = threading.Thread(target=self.monitorReceive)
        self.monitorThread.daemon = True
        self.monitorThread.start()
        rospy.sleep(startupDelay)
        self.sendCommand("RESET")
        rospy.sleep(0.2)
        self.sendCommand("START")

    def initializeReceiveMonitor(self, clock):
        self.receiveClock = clock
        self.receiveStarted = clock()
        self.receiveLock = threading.Lock()
        self.monitorStop = threading.Event()
        self.lastReceived = dict(bytes=None, OTOS=None, HMC=None)
        self.receivedCounts = dict(bytes=0, OTOS=0, HMC=0, invalid=0)
        self.lastArduinoStatus = "none"
        self.statusLoggedAt = {}
        self.receiveWarnings = {}
        self.receiveReportedHealthy = False
        self.receiveEverHealthy = False

    def monitorReceive(self):
        while not self.monitorStop.wait(0.2):
            self.checkReceiveHealth()

    def noteReceived(self, kind, count=1):
        with self.receiveLock:
            if kind in self.lastReceived:
                self.lastReceived[kind] = self.receiveClock()
            self.receivedCounts[kind] += count

    def checkReceiveHealth(self):
        now = self.receiveClock()
        with self.receiveLock:
            last = dict(self.lastReceived)
            counts = dict(self.receivedCounts)
            status = self.lastArduinoStatus
        bothSeen = last['OTOS'] is not None and last['HMC'] is not None
        if not self.receiveEverHealthy and not bothSeen and now-self.receiveStarted < self.startupGrace:
            return
        ages = {key: now-(self.receiveStarted if value is None else value)
                for key, value in last.items()}
        if ages['bytes'] >= self.receiveTimeout:
            problems = ['bytes']
        else:
            problems = [key for key in ('OTOS', 'HMC') if ages[key] >= self.receiveTimeout]
        cleared = set(self.receiveWarnings)-set(problems)
        resolved = [key for key in cleared if ages[key] < self.receiveTimeout]
        for key in cleared:
            del self.receiveWarnings[key]
        if not problems:
            self.receiveEverHealthy = True
            if not self.receiveReportedHealthy:
                rospy.loginfo("Serial sensor stream healthy on %s: valid OTOS and HMC received", self.port)
                self.receiveReportedHealthy = True
            return
        self.receiveReportedHealthy = False
        if resolved:
            rospy.loginfo("Serial receive resumed on %s for %s; still waiting for %s",
                          self.port, ','.join(sorted(resolved)), ','.join(problems))
        for key in problems:
            previous = self.receiveWarnings.get(key)
            if previous is not None and now-previous < self.warningInterval:
                continue
            self.receiveWarnings[key] = now
            if key == 'bytes':
                rospy.logwarn("Serial receive timeout on %s: no bytes for %.1f s; "
                              "OTOS=%d HMC=%d invalid=%d last_status=%s. "
                              "Check Arduino/USB; if resetting Arduino, restart Launch afterward.",
                              self.port, ages[key], counts['OTOS'], counts['HMC'], counts['invalid'], status)
            else:
                rospy.logwarn("%s receive timeout on %s: no valid %s record for %.1f s "
                              "while serial bytes are arriving; bytes=%d OTOS=%d HMC=%d "
                              "invalid=%d last_status=%s",
                              key, self.port, key, ages[key], counts['bytes'], counts['OTOS'],
                              counts['HMC'], counts['invalid'], status)

    def run(self):
        while not rospy.is_shutdown():
            self.sendPingIfNeeded()

            try:
                chunk = self.serial.readline(self.MAX_SERIAL_LINE_BYTES + 1)
            except serial.SerialException as exc:
                self.serialBuffer = b""
                self.discardSerialLine = True
                rospy.logerr_throttle(2.0, "Serial read failed: %s", exc)
                rospy.sleep(0.2)
                continue

            self.handleSerialChunk(chunk)

    def handleSerialChunk(self, chunk):
        # readline() can return a partial record on timeout. Keep it until LF,
        # including across empty reads, rather than parsing a truncated number.
        if not chunk:
            return

        self.noteReceived('bytes', len(chunk))

        if self.discardSerialLine:
            if chunk.endswith(b"\n"):
                self.discardSerialLine = False
            return

        self.serialBuffer += chunk
        if len(self.serialBuffer) > self.MAX_SERIAL_LINE_BYTES:
            self.serialBuffer = b""
            self.discardSerialLine = not chunk.endswith(b"\n")
            self.noteReceived('invalid')
            rospy.logwarn_throttle(2.0, "Discarding overlong serial line")
            return

        if not chunk.endswith(b"\n"):
            return

        line = self.serialBuffer.decode("utf-8", "replace").strip()
        self.serialBuffer = b""
        if line:
            self.handleLine(line)

    def sendCommand(self, command):
        try:
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()
        except serial.SerialException as exc:
            rospy.logerr_throttle(2.0, "Serial write failed on %s while sending %s: %s",
                                 self.port, command, exc)

    def sendPingIfNeeded(self):
        now = rospy.Time.now()
        if now - self.lastPingTime >= self.pingInterval:
            self.sendCommand("PING")
            self.lastPingTime = now

    def onShutdown(self):
        self.monitorStop.set()
        self.sendCommand("STOP")
        try:
            self.serial.close()
        except serial.SerialException:
            pass

    def handleLine(self, line):
        parts = line.split(",")
        messageType = parts[0]

        if messageType == "STATUS":
            status = ",".join(parts[1:])
            with self.receiveLock:
                self.lastArduinoStatus = status
            if parts[1:] and parts[1] in ("OTOS_INIT_FAILED", "HMC_INIT_FAILED", "NO_SENSORS_READY"):
                # Throttle each status independently so adjacent sensor failures
                # cannot suppress each other's initial report.
                now = self.receiveClock()
                previous = self.statusLoggedAt.get(status)
                if previous is None or now-previous >= self.warningInterval:
                    rospy.logerr("Arduino sensor initialization error: %s", status)
                    self.statusLoggedAt[status] = now
            elif parts[1:] and parts[1] in ("BRIDGE_TIMEOUT", "STREAM_OFF", "UNKNOWN_COMMAND"):
                rospy.logwarn("Arduino streaming warning: %s", status)
            elif parts[1:] and parts[1] in ("OTOS_READY", "HMC_READY", "RESET_DONE", "STREAM_ON"):
                rospy.loginfo("Arduino status: %s", status)
            else:
                rospy.loginfo_throttle(5.0, "Arduino status: %s", status)
            return

        if messageType == "ERROR":
            self.noteReceived('invalid')
            rospy.logerr_throttle(2.0, "Arduino error: %s", ",".join(parts[1:]))
            return

        stamp = rospy.Time.now()

        try:
            if messageType == "OTOS" and len(parts) == 5:
                self.handleOtos(parts, stamp)
                self.noteReceived('OTOS')
                return

            if messageType == "HMC" and len(parts) == 8:
                self.handleHmc(parts, stamp)
                self.noteReceived('HMC')
                return
        except ValueError as exc:
            self.noteReceived('invalid')
            rospy.logwarn_throttle(2.0, "Failed to parse line '%s': %s", line, exc)
            return

        self.noteReceived('invalid')
        rospy.logwarn_throttle(2.0, "Unexpected serial line: %s", line)

    def handleOtos(self, parts, stamp):
        _arduinoMs, xText, yText, headingText = parts[1:]
        x = float(xText)
        y = float(yText)
        heading = float(headingText)
        self.requireFinite((x, y, heading))

        poseMessage = Pose2D()
        poseMessage.x = x
        poseMessage.y = y
        poseMessage.theta = heading
        self.posePublisher.publish(poseMessage)

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, heading)
        self.tfBroadcaster.sendTransform(
            (x, y, 0.0),
            quaternion,
            stamp,
            self.otosBaseLinkFrame,
            self.otosOdomFrame,
        )

    def handleHmc(self, parts, stamp):
        (
            _arduinoMs,
            rollText,
            pitchText,
            headingText,
            axText,
            ayText,
            azText,
        ) = parts[1:]

        roll, pitch, heading, ax, ay, az = map(float, (rollText, pitchText, headingText, axText, ayText, azText))
        self.requireFinite((roll, pitch, heading, ax, ay, az))

        rpyMessage = Vector3Stamped()
        rpyMessage.header.stamp = stamp
        rpyMessage.header.frame_id = self.imuFrame
        rpyMessage.vector.x = roll
        rpyMessage.vector.y = pitch
        rpyMessage.vector.z = heading
        self.rpyPublisher.publish(rpyMessage)

        accelMessage = Vector3Stamped()
        accelMessage.header.stamp = stamp
        accelMessage.header.frame_id = self.imuFrame
        accelMessage.vector.x = ax
        accelMessage.vector.y = ay
        accelMessage.vector.z = az
        self.accelPublisher.publish(accelMessage)

    @staticmethod
    def requireFinite(values):
        if any(math.isnan(value) or math.isinf(value) for value in values):
            raise ValueError("sensor values must be finite")


if __name__ == "__main__":
    rospy.init_node("serial_sensor_bridge")

    try:
        bridge = SerialSensorBridge()
        bridge.run()
    except serial.SerialException as exc:
        rospy.logfatal("Failed to open serial port: %s", exc)
    except rospy.ROSInterruptException:
        pass

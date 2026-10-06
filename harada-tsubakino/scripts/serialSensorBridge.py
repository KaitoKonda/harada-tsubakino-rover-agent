#!/usr/bin/env python2

import rospy
import serial
import tf
from geometry_msgs.msg import Pose2D, Vector3Stamped


class SerialSensorBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyACM0")
        baud = rospy.get_param("~baud", 115200)
        timeout = rospy.get_param("~timeout", 0.2)
        startupDelay = rospy.get_param("~startup_delay", 2.0)
        pingInterval = rospy.get_param("~ping_interval", 0.3)

        self.otosOdomFrame = rospy.get_param("~otos_odom_frame", "otos_odom")
        self.otosBaseLinkFrame = rospy.get_param(
            "~otos_base_link_frame", "otos_base_link"
        )
        self.imuFrame = rospy.get_param("~imu_frame", "hmc6343_link")
        self.pingInterval = rospy.Duration(pingInterval)
        self.lastPingTime = rospy.Time(0)

        self.posePublisher = rospy.Publisher("odometry/otos", Pose2D, queue_size=20)
        self.rpyPublisher = rospy.Publisher("hmc6343_rpy", Vector3Stamped, queue_size=10)
        self.accelPublisher = rospy.Publisher("hmc6343_accel", Vector3Stamped, queue_size=10)

        self.tfBroadcaster = tf.TransformBroadcaster()
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        rospy.on_shutdown(self.onShutdown)

        rospy.loginfo("Opened serial port %s at %d baud", port, baud)
        rospy.sleep(startupDelay)
        self.sendCommand("RESET")
        rospy.sleep(0.2)
        self.sendCommand("START")

    def run(self):
        while not rospy.is_shutdown():
            self.sendPingIfNeeded()

            try:
                line = self.serial.readline()
            except serial.SerialException as exc:
                rospy.logerr_throttle(2.0, "Serial read failed: %s", exc)
                rospy.sleep(0.2)
                continue

            if not isinstance(line, str):
                try:
                    line = line.decode("utf-8", "replace")
                except AttributeError:
                    line = str(line)

            line = line.strip()

            if not line:
                continue

            self.handleLine(line)

    def sendCommand(self, command):
        try:
            self.serial.write(command + "\n")
            self.serial.flush()
        except serial.SerialException as exc:
            rospy.logerr_throttle(2.0, "Serial write failed: %s", exc)

    def sendPingIfNeeded(self):
        now = rospy.Time.now()
        if now - self.lastPingTime >= self.pingInterval:
            self.sendCommand("PING")
            self.lastPingTime = now

    def onShutdown(self):
        self.sendCommand("STOP")
        try:
            self.serial.close()
        except serial.SerialException:
            pass

    def handleLine(self, line):
        parts = line.split(",")
        messageType = parts[0]

        if messageType == "STATUS":
            rospy.loginfo_throttle(5.0, "Arduino status: %s", ",".join(parts[1:]))
            return

        stamp = rospy.Time.now()

        try:
            if messageType == "OTOS" and len(parts) == 5:
                self.handleOtos(parts, stamp)
                return

            if messageType == "HMC" and len(parts) == 8:
                self.handleHmc(parts, stamp)
                return
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse line '%s': %s", line, exc)
            return

        rospy.logwarn_throttle(2.0, "Unexpected serial line: %s", line)

    def handleOtos(self, parts, stamp):
        _arduinoMs, xText, yText, headingText = parts[1:]
        x = float(xText)
        y = float(yText)
        heading = float(headingText)

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

        rpyMessage = Vector3Stamped()
        rpyMessage.header.stamp = stamp
        rpyMessage.header.frame_id = self.imuFrame
        rpyMessage.vector.x = float(rollText)
        rpyMessage.vector.y = float(pitchText)
        rpyMessage.vector.z = float(headingText)
        self.rpyPublisher.publish(rpyMessage)

        accelMessage = Vector3Stamped()
        accelMessage.header.stamp = stamp
        accelMessage.header.frame_id = self.imuFrame
        accelMessage.vector.x = float(axText)
        accelMessage.vector.y = float(ayText)
        accelMessage.vector.z = float(azText)
        self.accelPublisher.publish(accelMessage)


if __name__ == "__main__":
    rospy.init_node("serial_sensor_bridge")

    try:
        bridge = SerialSensorBridge()
        bridge.run()
    except serial.SerialException as exc:
        rospy.logfatal("Failed to open serial port: %s", exc)
    except rospy.ROSInterruptException:
        pass

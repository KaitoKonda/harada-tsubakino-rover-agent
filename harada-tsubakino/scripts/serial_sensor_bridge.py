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
        startup_delay = rospy.get_param("~startup_delay", 2.0)
        ping_interval = rospy.get_param("~ping_interval", 0.3)

        self.otos_odom_frame = rospy.get_param("~otos_odom_frame", "otos_odom")
        self.otos_base_link_frame = rospy.get_param(
            "~otos_base_link_frame", "otos_base_link"
        )
        self.imu_frame = rospy.get_param("~imu_frame", "hmc6343_link")
        self.ping_interval = rospy.Duration(ping_interval)
        self.last_ping_time = rospy.Time(0)

        self.pose_pub = rospy.Publisher("otos_pose", Pose2D, queue_size=20)
        self.rpy_pub = rospy.Publisher("hmc6343_rpy", Vector3Stamped, queue_size=10)
        self.accel_pub = rospy.Publisher("hmc6343_accel", Vector3Stamped, queue_size=10)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("Opened serial port %s at %d baud", port, baud)
        rospy.sleep(startup_delay)
        self.send_command("RESET")
        rospy.sleep(0.2)
        self.send_command("START")

    def run(self):
        while not rospy.is_shutdown():
            self.send_ping_if_needed()

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

            self.handle_line(line)

    def send_command(self, command):
        try:
            self.serial.write(command + "\n")
            self.serial.flush()
        except serial.SerialException as exc:
            rospy.logerr_throttle(2.0, "Serial write failed: %s", exc)

    def send_ping_if_needed(self):
        now = rospy.Time.now()
        if now - self.last_ping_time >= self.ping_interval:
            self.send_command("PING")
            self.last_ping_time = now

    def on_shutdown(self):
        self.send_command("STOP")
        try:
            self.serial.close()
        except serial.SerialException:
            pass

    def handle_line(self, line):
        parts = line.split(",")
        msg_type = parts[0]

        if msg_type == "STATUS":
            rospy.loginfo_throttle(5.0, "Arduino status: %s", ",".join(parts[1:]))
            return

        stamp = rospy.Time.now()

        try:
            if msg_type == "OTOS" and len(parts) == 5:
                self.handle_otos(parts, stamp)
                return

            if msg_type == "HMC" and len(parts) == 8:
                self.handle_hmc(parts, stamp)
                return
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse line '%s': %s", line, exc)
            return

        rospy.logwarn_throttle(2.0, "Unexpected serial line: %s", line)

    def handle_otos(self, parts, stamp):
        _arduino_ms, x_str, y_str, heading_str = parts[1:]
        x = float(x_str)
        y = float(y_str)
        heading = float(heading_str)

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = heading
        self.pose_pub.publish(pose_msg)

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, heading)
        self.tf_broadcaster.sendTransform(
            (x, y, 0.0),
            quaternion,
            stamp,
            self.otos_base_link_frame,
            self.otos_odom_frame,
        )

    def handle_hmc(self, parts, stamp):
        (
            _arduino_ms,
            roll_str,
            pitch_str,
            heading_str,
            ax_str,
            ay_str,
            az_str,
        ) = parts[1:]

        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = stamp
        rpy_msg.header.frame_id = self.imu_frame
        rpy_msg.vector.x = float(roll_str)
        rpy_msg.vector.y = float(pitch_str)
        rpy_msg.vector.z = float(heading_str)
        self.rpy_pub.publish(rpy_msg)

        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = self.imu_frame
        accel_msg.vector.x = float(ax_str)
        accel_msg.vector.y = float(ay_str)
        accel_msg.vector.z = float(az_str)
        self.accel_pub.publish(accel_msg)


if __name__ == "__main__":
    rospy.init_node("serial_sensor_bridge")

    try:
        bridge = SerialSensorBridge()
        bridge.run()
    except serial.SerialException as exc:
        rospy.logfatal("Failed to open serial port: %s", exc)
    except rospy.ROSInterruptException:
        pass

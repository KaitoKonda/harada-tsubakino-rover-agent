#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーのオドメトリを取得するためのノードです。

import rospy
import sys
from lightrover_ros.srv import *
import time
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion ,TransformStamped
import tf
from rover_drive_trace.trace import (NullTrace, create_trace, difference, timed_call,
                                    create_service_proxy)
from nav_msgs.msg import Odometry

MS32_M_POS0 = 0x60
MS32_M_POS1 = 0x64

pre_count = [0.0, 0.0]

diff_count = [0, 0]

DIFF_COUNT_LIMIT = 1048575
ENCODER_COUNTER_BITS = 32
ENCODER_COUNTER_MODULUS = 1 << ENCODER_COUNTER_BITS
ENCODER_COUNTER_HALF_RANGE = ENCODER_COUNTER_MODULUS >> 1

pre_time = time.time()
diff_time = 0

WHEEL_CIRCUMFERENCE = 60.0*math.pi/1000
ENC_COUNTS_PER_TURN = 1188.024
ENC_PER_M = ENC_COUNTS_PER_TURN/WHEEL_CIRCUMFERENCE

ROVER_D = 0.143/2.0

x = 0.0
y = 0.0
th = 0.0

read_enc = create_service_proxy(rospy, 'wrc201_i2c', Wrc201Msg, lambda: trace)
trace = NullTrace()
sample_seq = 0
read_records = []
last_read_counts = None
last_read_error = None

def encoder_delta_counts(current, previous):
        # Smallest signed displacement on the 32-bit counter ring.
        # Fewer than 2**31 counts may elapse between samples.
        return ((int(current) - int(previous) + ENCODER_COUNTER_HALF_RANGE) %
                ENCODER_COUNTER_MODULUS) - ENCODER_COUNTER_HALF_RANGE

def getEncVal():
        global read_records, last_read_counts, last_read_error
        read_records = [] if trace.detailed else None
        last_read_counts = None
        last_read_error = None

        try:
                #VS-WRC201上のマイコンからエンコーダ値を取得
                enc_a = timed_call(trace, read_enc, (MS32_M_POS0,0,4,'r'), read_records)
                enc_b = timed_call(trace, read_enc, (MS32_M_POS1,0,4,'r'), read_records)

                if trace.detailed:
                        last_read_counts = [enc_a.readData, enc_b.readData]
                return enc_a.readData,enc_b.readData

        except rospy.ServiceException as e:
                if trace.detailed:
                        last_read_error = str(e)
                rospy.loginfo('Service call failed: %s',e)

def _calSpeed():
        global diff_time
        global pre_time
        global diff_count
        now_time = time.time()
        diff_time = now_time-pre_time
        pre_time = now_time

        global pre_count

        enc_val = getEncVal()
        if enc_val is None:
                print('None')
                return None

        #以前のエンコーダ値と現在のエンコーダ値の差を算出
        for i in range(2):
                delta = encoder_delta_counts(enc_val[i], pre_count[i])
                if(abs(delta) < DIFF_COUNT_LIMIT):
                        diff_count[i] = -1.0 * delta

        pre_count = enc_val

        #各タイヤの移動距離を算出
        distance = [float(diff_count[0])/ENC_PER_M,float(diff_count[1])/ENC_PER_M]
        #各タイヤの回転速度を算出
        speed = [distance[0]/diff_time, distance[1]/diff_time]

        #本体の直進・旋回速度を算出
        linear_x = ((speed[0] - speed[1])/2.0)
        angular_z = -1.0 * ((speed[0] + speed[1])/(2.0*ROVER_D))

        return linear_x, angular_z

def calSpeed():
        global sample_seq
        sample_seq += 1
        started = trace.now()
        previous_counts = list(pre_count) if trace.detailed else None
        result = None
        error_text = None
        try:
                result = _calSpeed()
                return result
        except Exception as error:
                if trace.detailed:
                        error_text = str(error)
                raise
        finally:
                if trace.enabled:
                        ended = trace.now()
                        trace.count('wheel_samples')
                        if result is None:
                                trace.count('wheel_sample_failures')
                        trace.observe('wheel_sample_duration_s', difference(ended, started))
                        trace.observe('wheel_calculation_dt_s', diff_time)
                if trace.detailed:
                        raw_delta = (None if last_read_counts is None else
                            [last_read_counts[i] - previous_counts[i] for i in range(2)])
                        wrapped_delta = (None if last_read_counts is None else
                            [encoder_delta_counts(last_read_counts[i], previous_counts[i])
                             for i in range(2)])
                        trace.emit('wheel_sample', dict(
                            sample_seq=sample_seq, previous_counts=previous_counts,
                            encoder_counts=last_read_counts,
                            raw_delta_counts=raw_delta,
                            wrapped_delta_counts=wrapped_delta,
                            delta_accepted=(None if wrapped_delta is None else
                                [abs(v) < DIFF_COUNT_LIMIT for v in wrapped_delta]),
                            used_delta_counts=list(diff_count), calculation_dt_s=diff_time,
                            rover_velocity=result, success=result is not None,
                            error=error_text or last_read_error, service_calls=read_records,
                            end_monotonic_s=ended, duration_s=difference(ended, started)),
                            at=started)

def cal_odometry(vx, vth):
        global x,y,th
        delta_x = vx*math.cos(th)*diff_time
        delta_y = vx*math.sin(th)*diff_time
        delta_th = vth*diff_time

        x += delta_x
        y += delta_y
        th += delta_th

def lightrover_odometry():
        rospy.init_node('wrc201_odometry', anonymous=True)
        global trace, read_enc
        read_enc = create_service_proxy(rospy, 'wrc201_i2c', Wrc201Msg,
            lambda: trace, persistent=rospy.get_param('~i2c_persistent', True))
        trace = create_trace(rospy, 'wheel', dict(
            upstream_blob='8fd864156607c2a2a87455a27bea5b210ef1f1a7',
            wheel_order=['motor0', 'motor1'],
            wheel_circumference_m=WHEEL_CIRCUMFERENCE,
            encoder_counts_per_turn=ENC_COUNTS_PER_TURN,
            encoder_counts_per_m=ENC_PER_M, track_width_m=2*ROVER_D,
            requested_rate_hz=100, diff_count_limit=DIFF_COUNT_LIMIT,
            encoder_delta_mode='signed_modulo_32',
            service_proxy_persistent=read_enc.persistent,
            service_lookup_timing='ServiceProxy._get_service_uri',
            additional_i2c_reads=False))
        rospy.wait_for_service('wrc201_i2c')

        odom_pub = rospy.Publisher('odom',Odometry, queue_size=50)
        odom_br = tf.TransformBroadcaster()

        rate = rospy.Rate(100)

        get_val = calSpeed()

        while not rospy.is_shutdown():
                get_val = calSpeed()
                if(get_val is None):
                        continue
                else:
                        cal_odometry(get_val[0],get_val[1])

                        now_time = rospy.Time.now()

                        odom_tf = TransformStamped()
                        odom_tf.header.stamp = now_time
                        odom_tf.header.frame_id = "odom"
                        odom_tf.child_frame_id = "base_link"

                        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

                        odom_tf.transform.translation.x = x
                        odom_tf.transform.translation.y = y
                        odom_tf.transform.translation.z = 0
                        odom_tf.transform.rotation = odom_quat

                        odom_br.sendTransform((x,y,0),
                                        odom_tf.transform.rotation,
                                        now_time,
                                        odom_tf.child_frame_id,
                                        odom_tf.header.frame_id)

                        odom = Odometry()
                        odom.header.stamp = now_time

                        odom.header.frame_id = "odom"

                        odom.pose.pose.position.x = x
                        odom.pose.pose.position.y = y
                        odom.pose.pose.position.z = 0
                        odom.pose.pose.orientation.x = odom_quat[0]
                        odom.pose.pose.orientation.y = odom_quat[1]
                        odom.pose.pose.orientation.z = odom_quat[2]
                        odom.pose.pose.orientation.w = odom_quat[3]

                        odom.child_frame_id = "base_link"
                        odom.twist.twist.linear.x = get_val[0]
                        odom.twist.twist.linear.y = 0.0
                        odom.twist.twist.angular.z = get_val[1]

                        odom_pub.publish(odom)
                        if trace.enabled:
                                trace.count('wheel_published')
                        if trace.detailed:
                                trace.emit('wheel_published', dict(
                                    sample_seq=sample_seq,
                                    odom_stamp_ns=now_time.to_nsec(),
                                    pose=[x,y,th], rover_velocity=list(get_val)))

                        rate.sleep()

if __name__=="__main__":
        rospy.loginfo('Start calculate odometry')
        try:
                lightrover_odometry()
        except rospy.ROSInterruptException:
                pass

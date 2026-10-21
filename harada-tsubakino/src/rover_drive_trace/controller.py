#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーを位置制御するためのノードです。

import rospy
import sys
from lightrover_ros.srv import *
import time
import math
from rover_drive_trace import motor as vs_wrc201_motor
from rover_drive_trace.trace import NullTrace, create_trace, difference, timed_call
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

write_msg = rospy.ServiceProxy('wrc201_i2c',Wrc201Msg)

#メモリマップアドレス
MU8_O_EN = 0x10
MU8_TRIG = 0x11
MS16_FB_PG0 = 0x20
MS16_FB_PG1 = 0x22

MS32_A_POS0 = 0x48
MS32_A_POS1 = 0x4c

MS16_T_OUT0 = 0x50
MS16_T_OUT1 = 0x52

MU16_FB_PCH0 = 0x30
MU16_FB_PCH1 = 0x32

linear_x = 0.0
angular_z = 0.0

current_v = [0.0, 0.0]
target_rover_v = [0.0, 0.0]

#車輪間距離の半分
ROVER_D = 0.143/2.0

motor_controller = vs_wrc201_motor.VsWrc201Motor()
trace = NullTrace()
command_seq = 0
latest_command = None
cycle_seq = 0
previous_cycle_at = None

def cb_get_rover_v(data):
        global linear_x,angular_z,current_v,ROVER_D,target_rover_v
        global cycle_seq, previous_cycle_at
        started = trace.now()
        cycle_seq += 1
        command_at_start = latest_command
        target_at_start = list(target_rover_v)
        period = difference(started, previous_cycle_at)
        previous_cycle_at = started
        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z

        #現在の直進・旋回速度から左右モータの現在の回転速度を算出
        current_v[1] = (linear_x + ROVER_D * angular_z)
        current_v[0] = -1.0 * (linear_x - ROVER_D * angular_z)

        #現在のモータ回転速度と目標のモータ回転速度をPOSコントローラに入力
        #左右モータへの出力値を算出
        output = None
        writes = []
        success = False
        error_text = None
        try:
                output = motor_controller.pos_controll(current_v, target_rover_v)
                drive_motor(output[0],output[1], writes)
                success = True
        except Exception as error:
                error_text = str(error)
                raise
        finally:
                if trace.enabled:
                        ended = trace.now()
                        trace.emit('control_cycle', dict(
                            cycle_seq=cycle_seq, period_s=period,
                            command_at_start=command_at_start,
                            command_seq_at_end=command_seq,
                            command_age_s=(None if command_at_start is None else
                                difference(started, command_at_start['received_monotonic_s'])),
                            wheel_odom_stamp_ns=data.header.stamp.to_nsec(),
                            wheel_odom_seq=data.header.seq,
                            measured_wheel_mps=list(current_v),
                            target_wheel_mps_at_start=target_at_start,
                            target_wheel_mps_at_end=list(target_rover_v),
                            ramped_wheel_mps=list(motor_controller.v_com),
                            acceleration_dt_s=getattr(motor_controller, 'trace_acceleration_dt', None),
                            integration_dt_s=getattr(motor_controller, 'trace_position_dt', None),
                            increment_counts=output,
                            fractional_counts=list(motor_controller.buf_enc_com),
                            service_calls=writes, success=success, error=error_text,
                            end_monotonic_s=ended, duration_s=difference(ended, started)),
                            at=started)

def cb_set_target_v(data):
        global ROVER_D,target_rover_v,command_seq,latest_command
        received = trace.now()
        #目標直進・旋回速度から左右モータの目標回転速度を算出
        target_rover_v[1] = (data.linear.x + ROVER_D * data.angular.z)
        target_rover_v[0] = -1.0 * (data.linear.x - ROVER_D * data.angular.z)
        command_seq += 1
        latest_command = dict(command_seq=command_seq,
            received_monotonic_s=received, linear_mps=data.linear.x,
            angular_radps=data.angular.z,
            caller_id=(getattr(data, '_connection_header', None) or {}).get('callerid'))
        if trace.enabled:
                trace.emit('command_received', dict(latest_command,
                    target_wheel_mps=list(target_rover_v)), at=received)

def drive_motor(r_speed, l_speed, records=None):
        if records is None:
                records = []
        timed_call(trace, write_msg, (MS32_A_POS0,r_speed,4,'w'), records)
        timed_call(trace, write_msg, (MS32_A_POS1,l_speed,4,'w'), records)
        timed_call(trace, write_msg, (MU8_TRIG,0x03,1,'w'), records)

def init_write(*args):
        records = []
        try:
                return timed_call(trace, write_msg, args, records)
        finally:
                if trace.enabled:
                        trace.emit('initialization_write', dict(service_calls=records))

def pos_cntrl():
        rospy.init_node('pos_controller', anonymous=True)
        global trace
        trace = create_trace(rospy, 'controller', dict(
            upstream_blob='618bb67db1044e75fbde482903250f14aa94adb9',
            motor_upstream_blob='599c2eedeec64bc68008ad2aedce1f45e3dfe8d3',
            wheel_order=['motor0', 'motor1'], track_width_m=2*ROVER_D,
            position_p_gain=[128, 128], punch=[2500, 2500],
            motor_parameters=list(motor_controller.motor_param),
            encoder_counts_per_m=motor_controller.ENC_PER_M,
            additional_i2c_reads=False))
        rospy.wait_for_service('wrc201_i2c')

        rospy.loginfo('Start POS Controll')

        init_write(MU8_O_EN,0x00,1,'w')          #モータ出力禁止
        init_write(MU8_TRIG,0x0c,1,'w')          #エンコーダリセット
        init_write(MS16_FB_PG0,0x0080,2,'w')     #モータ0位置補償Pゲイン設定
        init_write(MS16_FB_PG1,0x0080,2,'w')     #モータ1位置補償Pゲイン設定
        init_write(MU16_FB_PCH0,0x09C4,2,'w')    #モータ0最低出力値設定
        init_write(MU16_FB_PCH1,0x09C4,2,'w')    #モータ1最低出力値設定
        init_write(MU8_O_EN,0x03,1,'w')          #モータ出力許可

        rospy.Subscriber('odom',Odometry,cb_get_rover_v)
        rospy.Subscriber('rover_drive',Twist,cb_set_target_v)

        rospy.spin()

if __name__ == '__main__':
        pos_cntrl()

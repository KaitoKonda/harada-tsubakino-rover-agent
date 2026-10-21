#!/usr/bin/env python2
import rospy
from rover_drive_trace.odometry import lightrover_odometry

if __name__ == '__main__':
    rospy.loginfo('Start calculate odometry')
    try:
        lightrover_odometry()
    except rospy.ROSInterruptException:
        pass

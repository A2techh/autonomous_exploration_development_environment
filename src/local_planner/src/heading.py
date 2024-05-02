#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def imu_callback1(msg):
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    #rospy.loginfo("ublox1 Euler Angles (degrees): Roll={:.2f}, Pitch={:.2f}, Yaw={:.2f}".format(roll_deg, pitch_deg, yaw_deg))

def imu_callback2(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    rospy.loginfo("ublox2 Euler Angles (degrees): Roll={}, Pitch={}, Yaw={}".format(roll_deg, pitch_deg, yaw_deg))

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imu_callback1)
    rospy.Subscriber("/state_estimation", Odometry, imu_callback2)
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass


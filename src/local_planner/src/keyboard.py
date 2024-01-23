#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty
import sys
import tty
import termios

# Function to read a single character from the terminal without pressing enter
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rospy.init_node('teleop_node')
    pub_cmd_vel = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
    pub_start_waypoints = rospy.Publisher('/start_waypoints', Empty, queue_size=1)

    print("Press 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right.")
    print("Press 'q' to exit.")

    try:
        while not rospy.is_shutdown():
            char = getch()

            cmd_vel_msg = TwistStamped()
            cmd_vel_msg.header.stamp = rospy.Time.now()

            if char == 'w':
                cmd_vel_msg.twist.linear.x = 0.5
            elif char == 's':
                cmd_vel_msg.twist.linear.x = -0.5
            elif char == 'a':
                cmd_vel_msg.twist.angular.z = 0.25
            elif char == 'd':
                cmd_vel_msg.twist.angular.z = -0.25
            elif char == 'x':
                print("stop")
            elif char == 'j':
                start_waypoints_msg = Empty()
                pub_start_waypoints.publish(start_waypoints_msg)
            elif char == 'q':
                break

            pub_cmd_vel.publish(cmd_vel_msg)

    except rospy.ROSInterruptException:
        pass
    finally:
        print("\nExiting teleop_node.")

if __name__ == '__main__':
    main()


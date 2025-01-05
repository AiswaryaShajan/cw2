#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('move_robot_node')
    cmd_vel_pub = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Move forward with 0.5 m/s
    move_cmd.angular.z = 0.0  # No rotation

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing cmd_vel: linear.x={move_cmd.linear.x}, angular.z={move_cmd.angular.z}")
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
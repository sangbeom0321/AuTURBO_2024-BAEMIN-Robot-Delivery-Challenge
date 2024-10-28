#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pow, sin
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, CollisionData

class cmd_6wheel_node:
    def __init__(self):
        # Set up ROS topics
        rospy.Subscriber('/cmd_vel', Twist ,self.callback)
        self.cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)

    def callback(self, msg):
        # Callback to receive the published path
        cmd_msg = SkidSteer6wUGVCtrlCmd()
        cmd_msg.cmd_type=3
        cmd_msg.Target_linear_velocity = msg.linear.x
        cmd_msg.Target_angular_velocity = msg.angular.z
        rospy.loginfo("Linear Velocity: %f, Angular Velocity: %f", cmd_msg.Target_linear_velocity, cmd_msg.Target_angular_velocity)
        self.cmd_pub.publish(cmd_msg)


if __name__ == '__main__':
    rospy.init_node('cmd_6wheel_node')
    controller = cmd_6wheel_node()
    rospy.spin()

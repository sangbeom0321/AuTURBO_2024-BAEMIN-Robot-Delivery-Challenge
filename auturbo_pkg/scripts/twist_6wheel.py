#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from math import sin, cos
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, CollisionData


class OdomPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vth = 0.0

        self.last_time = rospy.Time.now()
        self.last_cmd_vel_time = rospy.Time.now()
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.action_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)

    def cmd_vel_callback(self, data):
        self.vx = data.linear.x
        self.vth = data.angular.z * -1

        ci = SkidSteer6wUGVCtrlCmd()
        ci.cmd_type = 3
        ci.Target_linear_velocity = self.vx
        ci.Target_angular_velocity = self.vth
        self.action_pub.publish(ci)
        self.last_cmd_vel_time = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('odom_from_cmd_vel', anonymous=True)
    odom_publisher = OdomPublisher()
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        elapsed_time = rospy.Time.now() - odom_publisher.last_cmd_vel_time
        if elapsed_time > rospy.Duration(1):
            ci = SkidSteer6wUGVCtrlCmd()
            ci.cmd_type = 3
            ci.Target_linear_velocity = 0
            ci.Target_angular_velocity = 0
            odom_publisher.action_pub.publish(ci)
            odom_publisher.last_cmd_vel_time = rospy.Time.now()
        r.sleep()

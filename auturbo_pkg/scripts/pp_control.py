#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pow, sin
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, CollisionData
class PurePursuitController:
    def __init__(self):
        # Set up ROS topics
        self.path_sub = rospy.Subscriber('/published_path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=10)

        # Parameters
        self.lookahead_distance = 1  # Distance to the lookahead point
        self.linear_velocity = 2  # Default linear velocity
        self.wheelbase = 1  # Distance between front and rear axles for turning

        # Internal state
        self.path = None
        self.current_pose = None

    def path_callback(self, msg):
        # Callback to receive the published path
        self.path = msg.poses

    def odom_callback(self, msg):
        # Callback to receive the current odometry information
        self.current_pose = msg.pose.pose
        self.control_loop()

    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return

        # Extract the robot's current position and orientation
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Find the lookahead point on the path
        lookahead_point = self.find_lookahead_point(x, y)

        if lookahead_point is not None:
            # Compute the control commands
            dx = lookahead_point[0] - x
            dy = lookahead_point[1] - y
            angle_to_goal = atan2(dy, dx)
            angle_diff = angle_to_goal - yaw

            # Compute angular velocity using the curvature (steering angle calculation)
            angular_velocity = 2 * self.linear_velocity * sin(angle_diff) / self.lookahead_distance

            # Publish the control commands
            cmd_msg = SkidSteer6wUGVCtrlCmd()
            cmd_msg.cmd_type=3
            cmd_msg.Target_linear_velocity = self.linear_velocity
            cmd_msg.Target_angular_velocity = -angular_velocity
            self.cmd_pub.publish(cmd_msg)

    def find_lookahead_point(self, x, y):
        # Find the lookahead point along the path
        for pose in self.path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            distance = sqrt(pow(px - x, 2) + pow(py - y, 2))
            if distance >= self.lookahead_distance:
                return (px, py)
        return None

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_controller')
    controller = PurePursuitController()
    rospy.spin()
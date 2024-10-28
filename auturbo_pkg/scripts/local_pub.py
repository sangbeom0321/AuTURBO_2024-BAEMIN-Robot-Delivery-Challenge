#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import tf
import math
import numpy as np

class MapUpdater:
    def __init__(self):
        # Initialize variables
        self.map_received = False
        self.map_data_original = None  # Store the original map data
        self.map_info = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin = None
        self.robot_pose = Pose()

        # Initialize the transform listener
        self.tf_listener = tf.TransformListener()

        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan_hy', LaserScan, self.scan_callback)

        # Publisher for the updated map
        self.map_pub = rospy.Publisher('/updated_map', OccupancyGrid, queue_size=1)

    def map_callback(self, data):
        if not self.map_received:
            # Store original map data
            self.map_data_original = list(data.data)
            self.map_info = data.info
            self.map_width = data.info.width
            self.map_height = data.info.height
            self.map_resolution = data.info.resolution
            self.map_origin = data.info.origin
            self.map_received = True
            rospy.loginfo("Map received and stored.")
            # Unsubscribe from /map after receiving it once
            self.map_sub.unregister()

    def scan_callback(self, data):
        if not self.map_received:
            rospy.logwarn("Map not received yet.")
            return

        # Create a copy of the original map data for this scan
        map_data = list(self.map_data_original)

        try:
            # Get the robot's pose in the map frame
            now = rospy.Time(0)
            self.tf_listener.waitForTransform('map', 'velodyne', now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'velodyne', now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: %s", str(e))
            return

        robot_x, robot_y, _ = trans
        orientation_q = rot
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_q)
        print(yaw)
        # Process laser scan data
        ranges = data.ranges
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        # print(angle_min,angle_increment)
        for i in range(len(ranges)):
            range_i = ranges[i]
            if np.isinf(range_i) or np.isnan(range_i):
                continue
            # Calculate the angle of the current measurement
            angle = angle_min + i * angle_increment + yaw
            # Transform scan to world coordinates
            x = robot_x + range_i * math.cos(angle)
            y = robot_y + range_i * math.sin(angle)
            # Convert world coordinates to map indices
            map_x = int((x - self.map_origin.position.x) / self.map_resolution)
            map_y = int((y - self.map_origin.position.y) / self.map_resolution)
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                index = map_y * self.map_width + map_x
                map_data[index] = 100  # Mark cell as occupied

        # Publish the updated map
        updated_map = OccupancyGrid()
        updated_map.header.stamp = rospy.Time.now()
        updated_map.header.frame_id = 'map'
        updated_map.info = self.map_info
        updated_map.data = map_data
        self.map_pub.publish(updated_map)

if __name__ == '__main__':
    rospy.init_node('map_updater')
    MapUpdater()
    rospy.spin()

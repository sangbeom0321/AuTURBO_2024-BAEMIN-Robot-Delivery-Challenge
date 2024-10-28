#!/usr/bin/env python

import rospy
import json
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def load_json_file(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    return data

def json_to_path(json_data):
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.frame_id = "map"  # Change frame_id as needed
    path_msg.header.stamp = rospy.Time.now()

    for point in json_data:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()

        # Extract x, y, z from the JSON data
        pose_stamped.pose.position.x = point['x']
        pose_stamped.pose.position.y = point['y']
        pose_stamped.pose.position.z = point['z']

        # Optional: Set orientation if needed (currently set to identity quaternion)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        path_msg.poses.append(pose_stamped)

    return path_msg

def json_to_path_publisher():
    rospy.init_node('json_to_path_publisher', anonymous=True)
    
    # Load the JSON file from the parameter or hardcoded file path
    json_file_path = '/root/catkin_ws/src/auturbo_pkg/paths/3to3.json'  # Specify your JSON file path
    json_data = load_json_file(json_file_path)
    
    # Publisher to publish the path
    path_pub = rospy.Publisher('/published_path', Path, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz publishing rate
    while not rospy.is_shutdown():
        path_msg = json_to_path(json_data)
        path_pub.publish(path_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        json_to_path_publisher()
    except rospy.ROSInterruptException:
        pass
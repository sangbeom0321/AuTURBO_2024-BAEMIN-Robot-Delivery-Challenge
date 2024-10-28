#!/usr/bin/env python3

from PIL import Image
import numpy as np
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from scipy.ndimage import binary_dilation

def load_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def load_pgm_image(image_file):
    image = Image.open(image_file)
    return np.array(image)

def convert_map_data(map_data):

    map_converted = np.zeros(map_data.shape, dtype=np.int8)
    map_converted[map_data == 205] = 100  # Free
    map_converted[map_data < 128] = 0     # Occupied
    map_converted[map_data >= 128] = -1   # Unknown
    return map_converted

def inflate_map(map_data, inflation_radius):
    """
    Inflate the obstacles in the map by the given radius.
    """
    # Create a binary map where occupied cells are True
    occupied = map_data == 0

    # Create a structuring element for dilation
    structure = np.ones((2 * inflation_radius + 1, 2 * inflation_radius + 1))

    # Dilate the occupied cells
    inflated = binary_dilation(occupied, structure=structure).astype(np.bool_)

    # Update the map data: set inflated cells to 100 (occupied)
    map_data_inflated = np.copy(map_data)
    map_data_inflated[inflated] = 100

    return map_data_inflated

def publish_map(map_data, yaml_params):
    # Create a ROS publisher
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.init_node('map_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    # Create OccupancyGrid message
    map_msg = OccupancyGrid()

    # Set metadata
    map_msg.header.frame_id = "map"
    map_msg.info.resolution = yaml_params['resolution']
    map_msg.info.width = map_data.shape[1]
    map_msg.info.height = map_data.shape[0]

    # Set origin (from the yaml)
    map_msg.info.origin = Pose()
    map_msg.info.origin.position.x = yaml_params['origin'][0]
    map_msg.info.origin.position.y = yaml_params['origin'][1]
    map_msg.info.origin.position.z = 0
    map_msg.info.origin.orientation.w = 1.0

    # Flatten the map data and convert to ROS OccupancyGrid data format
    map_msg.data = (map_data.flatten()).tolist()

    # Keep publishing the map until the node is stopped
    while not rospy.is_shutdown():
        map_msg.header.stamp = rospy.Time.now()
        pub.publish(map_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Path to the yaml file
        yaml_file_path = '/root/mounted_folder/baemin_map_edited.yaml'
        yaml_params = load_yaml(yaml_file_path)

        # Path to the pgm file (from yaml file)
        pgm_file_path = yaml_params['image']

        # Load map image (PGM file)
        map_data = load_pgm_image(pgm_file_path)

        # Convert the map data to -1, 0, and 100 scale
        map_data_converted = convert_map_data(map_data)

        # Inflate the map
        inflation_radius = 5  # Adjust the radius as needed
        map_data_inflated = inflate_map(map_data_converted, inflation_radius)

        # Publish map
        publish_map(map_data_inflated, yaml_params)

    except rospy.ROSInterruptException:
        pass

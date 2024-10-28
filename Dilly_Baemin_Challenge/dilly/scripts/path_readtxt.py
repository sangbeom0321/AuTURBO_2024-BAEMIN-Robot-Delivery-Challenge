#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker

class TxtPublisherNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('txt_publisher_node', anonymous=True)

        # Publisher to send data
        self.pub = rospy.Publisher('txt_data', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(10)  
        # Subscriber to listen for goal achievement
        self.sub = rospy.Subscriber('goal_achieved', Bool, self.goal_callback)

        # File path to the 3to3_fix.txt file (update this to the correct path)
        # self.file_path = './path_list/test_sin3.txt'  # Update this to the correct path
        # self.file_path = './path_list/portal_test.txt'
        self.file_path = '/root/catkin_ws/src/Dilly_Baemin_Challenge/dilly/scripts/path_list/test_sin3.txt' 

        # Read the file lines into memory
        self.lines = self.read_file(self.file_path)
        self.current_index = 0  # To track the current line being processed
        # row_data = [float(x) for x in self.lines[self.current_index].split()]
        self.msg = Float32MultiArray()
        # self.msg.data = row_data
        # rospy.loginfo(f"Publishing: {self.msg.data}")
        # self.pub.publish(self.msg)

        # Control whether the next row can be published
        self.can_publish_next = True
        rospy.loginfo("TxtPublisherNode initialized")
        self.publish_next_row()
        
        # Wait for initial ROS setup to complete
        

    def read_file(self, file_path):
        """Read the file and return a list of lines with float data."""
        try:
            with open(file_path, 'r') as file:
                lines = [line.strip() for line in file if line.strip()]
            return lines
        except Exception as e:
            rospy.logerr(f"Error reading file: {e}")
            return []

    def goal_callback(self, goal_data):
        """Callback function triggered when the goal is achieved."""
        if goal_data.data:  # If goal is achieved
            rospy.loginfo("Goal achieved, publishing next data row...")
            self.can_publish_next = True
            self.publish_next_row()



    def publish_next_row(self):
        """Publish the next row of data if available."""
        if self.current_index < len(self.lines) and self.can_publish_next:
            # Extract the row and convert it to a list of floats
            row_data = [float(x) for x in self.lines[self.current_index].split()]
            print(row_data)
            # Create a Float32MultiArray message
            # msg = Float32MultiArray()
            self.msg.data = row_data

            # Publish the message
            rospy.loginfo(f"Publishing: {self.msg.data}")
            self.pub.publish(self.msg)

            # Update the index and reset publishing control
            self.current_index += 1
            self.can_publish_next = False  # Wait for the next goal achievement
        else:
            rospy.loginfo("All data published or waiting for goal achievement...")

    def run(self):
        """Keep the node running until shutdown."""
        while not rospy.is_shutdown():
            # print(self.msg.data)
            self.pub.publish(self.msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        node = TxtPublisherNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

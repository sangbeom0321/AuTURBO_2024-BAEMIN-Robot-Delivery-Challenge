#!/usr/bin/env python3

import rospy
from morai_msgs.srv import WoowaDillyEventCmdSrv
from morai_msgs.msg import DillyCmd, DillyCmdResponse, WoowaDillyStatus 
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class DeliveryNode:
    def __init__(self):
        rospy.init_node('DeliveryNode', anonymous=True)

        self.request = DillyCmd()
        self.response = DillyCmdResponse()

        self.Pickup = True
        self.service_call = False
        self.DillyStatusStop = False
        self.DillyDeliveryItem = []
        self.rate = rospy.Rate(3)  
        
        # subscribe
        self.dillystatus_sub = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, self.dillystatus_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # publish
        self.service_response_pub = rospy.Publisher('/DeliveryResponse', Bool, queue_size=5)
        self.Dillystop_pub = rospy.Publisher('/DillyDilveryStop', Bool, queue_size=5)

    def dillystatus_callback(self, delivery): # need to be change 
        # print(len(delivery.deliveryItem))
        self.DillyDeliveryItem = delivery.deliveryItem
        if len(delivery.deliveryItem) == 0:
            self.Pickup = True
        else:
            self.Pickup = False
            
    def imu_callback(self, imu_data):
        linear_acc = math.sqrt(imu_data.linear_acceleration.x**2 + imu_data.linear_acceleration.y**2) 
        angular_acc = math.sqrt(imu_data.angular_velocity.z**2)

        if linear_acc <0.1 and angular_acc <0.1:
            self.DillyStatusStop = True
        else:
            self.DillyStatusStop = False
    
    def odom_callback(self, odom_data):
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y

        DeliveryArea = math.sqrt((x - 65.2)**2 + (y - (44.1))**2 ) # need to be change
        # DeliveryArea = math.sqrt((x - 30.7)**2 + (y - (-41.1))**2)
        # DeliveryArea = math.sqrt((x - 372.1)**2 + (y - (-116.3))**2)
        # print(self.DillyStatusStop)
        # print(DeliveryArea)
        if DeliveryArea < 1.5 and (3 not in self.DillyDeliveryItem):
            self.Dillystop_pub.publish(Bool(data=True))
        else:
            self.Dillystop_pub.publish(Bool(data=False))

        if self.DillyStatusStop == True and DeliveryArea < 1.5:
            self.service_call = True
        else:
            self.service_call = False
            


    def run(self):
        while not rospy.is_shutdown():
            # print(self.service_result)
            if self.service_call:
                rospy.wait_for_service('/WoowaDillyEventCmd')
                try:
                    woowa_dilly_event_cmd = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)

                    # Create a request message
                    # print(self.Pickup)
                    self.request.isPickup = self.Pickup
                    if self.Pickup:
                        self.request.deliveryItemIndex = 3
                    else:
                        self.request.deliveryItemIndex = 3   
                        
                        # print(self.request)
                        # Call the service
                    self.response = woowa_dilly_event_cmd(self.request)
                    if self.response.response.result:
                        self.service_response_pub.publish(self.response.response.result)
                        rospy.loginfo("Service was successful. Result: True")
                    else:
                        self.service_response_pub.publish(self.response.response.result)
                        rospy.loginfo("Service was successful, but result is False")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s", str(e))
            else:
                continue
            self.rate.sleep() 

if __name__ == '__main__':
    try:
        deliverynode = DeliveryNode()
        deliverynode.run()
    except rospy.ROSInterruptException:
        pass


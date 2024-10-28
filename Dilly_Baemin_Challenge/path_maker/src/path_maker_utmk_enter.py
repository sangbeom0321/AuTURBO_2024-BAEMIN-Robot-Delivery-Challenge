#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import os
import sys
import rospy
import rospkg
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from math import pi, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import tf
import threading

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)

        self.cnt = 0
        self.is_gps = False
        self.prev_x = 0
        self.prev_y = 0

        self.x_offset = 334212.29
        self.y_offset = 4143082.44

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        full_path = pkg_path + '/' + self.path_folder_name + '/' + self.make_path_name + '.txt'
        self.f = open(full_path, 'w')

        # Start a thread to listen for Enter key press
        self.input_thread = threading.Thread(target=self.listen_for_enter)
        self.input_thread.daemon = True
        self.input_thread.start()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Main loop continues running, but path_make will be called by Enter press
            rate.sleep()

        self.f.close()

    def listen_for_enter(self):
        """Listen for Enter key press to trigger path recording"""
        while True:
            input("Press Enter to record path point...")
            if self.is_gps:  # Only make the path if GPS data is available
                self.path_make()
            else:
                print("Waiting for GPS data...")

    def path_make(self):
        x = self.xy_zone[0] - self.e_o
        y = self.xy_zone[1] - self.n_o
        z = 0

        # distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

        # if distance > 10:  # Record path if distance is greater than a threshold (e.g., 10 units)
        data = '{0}\t{1}\t{2}\n'.format(x, y, z)
        self.f.write(data)
        self.prev_x = x
        self.prev_y = y
        self.cnt += 1
        print(f"Point {self.cnt} recorded at: X: {x}, Y: {y}")

    def gpsCB(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.e_o = data.eastOffset
        self.n_o = data.northOffset
        self.xy_zone = self.proj_UTM(self.lon, self.lat)
        self.is_gps = True


if __name__ == '__main__':
    try:
        path_maker_ = PathMaker()
    except rospy.ROSInterruptException:
        pass

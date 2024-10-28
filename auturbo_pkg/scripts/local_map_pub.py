#!/usr/bin/env python3

import rospy
import tf
import math
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np

class LocalMapPublisher:
    def __init__(self):
        rospy.init_node('local_map_publisher')

        # Subscriber to the global map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Subscriber to the laser scan
        self.laser_sub = rospy.Subscriber('/scan_hy', LaserScan, self.laser_callback)
        
        # Publisher for the local map
        self.local_map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=10)
        
        # Transform listener to get robot pose
        self.tf_listener = tf.TransformListener()

        self.global_map = None
        self.local_map = None
        self.robot_pose = None
        self.resolution = 0.05  # Default resolution

        rospy.spin()

    def map_callback(self, msg):
        self.global_map = msg
        self.resolution = msg.info.resolution
        self.extract_local_map()

    def laser_callback(self, scan_msg):
        # 라이다 데이터 처리
        self.add_lidar_to_map(scan_msg)
        self.publish_local_map()

    def extract_local_map(self):
        if self.global_map is None:
            return
        
        try:
            # 로봇의 현재 위치와 방향 (orientation) 추출
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = self.quaternion_to_yaw(rot)

            # 로컬 맵의 크기 정의 (예: 10m x 10m)
            local_map_size = 20  # meters
            local_map_radius = int(local_map_size / self.resolution / 2)

            # 로컬 맵 초기화
            self.local_map = np.zeros((local_map_radius*2, local_map_radius*2), dtype=np.int8)

            # Global map의 원점과 크기 정보
            map_origin_x = self.global_map.info.origin.position.x
            map_origin_y = self.global_map.info.origin.position.y
            map_width = self.global_map.info.width
            map_height = self.global_map.info.height

            # 로봇의 위치에 해당하는 그리드 좌표 (글로벌 맵 좌표계에서)
            robot_grid_x = int((robot_x - map_origin_x) / self.resolution)
            robot_grid_y = int((robot_y - map_origin_y) / self.resolution)

            # 회전 변환 행렬 적용하여 글로벌 맵에서 로컬 맵으로 추출
            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)

            for i in range(-local_map_radius, local_map_radius):
                for j in range(-local_map_radius, local_map_radius):
                    # 로컬 좌표를 글로벌 좌표로 변환하면서 회전 적용
                    local_x = i * self.resolution
                    local_y = j * self.resolution

                    global_x = cos_yaw * local_x - sin_yaw * local_y + robot_x
                    global_y = sin_yaw * local_x + cos_yaw * local_y + robot_y

                    global_grid_x = int((global_x - map_origin_x) / self.resolution)
                    global_grid_y = int((global_y - map_origin_y) / self.resolution)

                    if 0 <= global_grid_x < map_width and 0 <= global_grid_y < map_height:
                        self.local_map[j + local_map_radius, i + local_map_radius] = self.global_map.data[global_grid_x + global_grid_y * map_width]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF error: Could not get robot pose")

    def add_lidar_to_map(self, scan_msg):
        if self.local_map is None:
            return
        
        try:
            # 라이다 데이터를 로봇의 base_footprint 좌표계를 기준으로 처리
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = self.quaternion_to_yaw(rot)

            angle_min = scan_msg.angle_min
            angle_increment = scan_msg.angle_increment

            for i, range_value in enumerate(scan_msg.ranges):
                if range_value < scan_msg.range_max:
                    angle = angle_min + i * angle_increment
                    
                    # 로봇 기준으로 장애물 좌표 계산 (라이다 데이터는 로봇을 기준으로 함)
                    obstacle_x = range_value * np.cos(angle)
                    obstacle_y = range_value * np.sin(angle)

                    # 좌표를 로컬 맵에 대각선으로 뒤집어서 적용 (X <-> Y 교환)
                    self.add_obstacle_to_local_map(obstacle_y, obstacle_x)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF error: Could not get robot pose")

    def add_obstacle_to_local_map(self, x, y):
        # 로봇 기준으로 라이다 데이터를 로컬 맵 좌표로 변환
        resolution = self.resolution

        # 로봇 중심 좌표계를 로컬 맵의 그리드 좌표로 변환
        local_grid_x = int(x / resolution) + self.local_map.shape[0] // 2
        local_grid_y = int(y / resolution) + self.local_map.shape[1] // 2

        if 0 <= local_grid_x < self.local_map.shape[0] and 0 <= local_grid_y < self.local_map.shape[1]:
            self.local_map[local_grid_x, local_grid_y] = 100  # 장애물 표시

    def publish_local_map(self):
        if self.local_map is None:
            return

        # OccupancyGrid 메시지로 변환
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = "base_footprint"  # 로컬 맵이 로봇 기준으로 설정됨

        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.local_map.shape[1]
        occupancy_grid.info.height = self.local_map.shape[0]

        # Local map의 원점을 로봇 위치로 설정 (base_footprint 좌표계 기준)
        occupancy_grid.info.origin.position.x = -self.local_map.shape[1] * self.resolution / 2
        occupancy_grid.info.origin.position.y = -self.local_map.shape[0] * self.resolution / 2
        occupancy_grid.info.origin.position.z = 0

        # Local map을 1차원 리스트로 변환
        occupancy_grid.data = self.local_map.flatten().tolist()

        # 발행
        self.local_map_pub.publish(occupancy_grid)

    def quaternion_to_yaw(self, quat):
        """Quaternion을 Yaw(회전 각도)로 변환하는 함수"""
        x, y, z, w = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

if __name__ == '__main__':
    try:
        LocalMapPublisher()
    except rospy.ROSInterruptException:
        pass

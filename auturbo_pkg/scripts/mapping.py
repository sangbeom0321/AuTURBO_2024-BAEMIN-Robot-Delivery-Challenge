#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import message_filters

class GridMapGenerator:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('grid_map_generator')

        # Odometry와 LaserScan 데이터를 동기화하여 구독
        odom_sub = message_filters.Subscriber('/odom', Odometry)
        scan_sub = message_filters.Subscriber('/scan', LaserScan)

        # ApproximateTimeSynchronizer: 약간의 시간 차이를 허용하여 메시지 동기화
        self.ts = message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # OccupancyGrid 메시지 발행
        self.map_pub = rospy.Publisher('/grid_map', OccupancyGrid, queue_size=10)

        # 맵 크기 및 해상도 설정 (OccupancyGrid)
        self.map_width = 500  # 셀의 개수
        self.map_height = 500
        self.map_resolution = 0.1  # 각 셀이 10cm x 10cm 해상도를 가짐
        self.origin_x = -25  # 세계 좌표에서 맵의 시작점 (x축)
        self.origin_y = -25  # 세계 좌표에서 맵의 시작점 (y축)

        # OccupancyGrid 맵 초기화
        self.grid_map = OccupancyGrid()
        self.grid_map.header.frame_id = "map"
        self.grid_map.info.resolution = self.map_resolution
        self.grid_map.info.width = self.map_width
        self.grid_map.info.height = self.map_height
        self.grid_map.info.origin.position.x = self.origin_x
        self.grid_map.info.origin.position.y = self.origin_y
        self.grid_map.info.origin.orientation.w = 1.0  # 회전 없음
        self.grid_map.data = [-1] * (self.map_width * self.map_height)  # 모든 셀을 -1(알 수 없음)로 초기화

    # 동기화된 콜백 함수
    def callback(self, odom, scan):
        # Odometry와 LaserScan 데이터를 받아서 처리

        # 1. Odometry 콜백: 로봇의 현재 위치와 방향을 갱신
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y

        # Orientation을 쿼터니언에서 오일러 각으로 변환 (yaw)
        quaternion = (
            odom.pose.pose.orientation.x, 
            odom.pose.pose.orientation.y, 
            odom.pose.pose.orientation.z, 
            odom.pose.pose.orientation.w
        )
        _, _, robot_yaw = euler_from_quaternion(quaternion)

        # 2. LaserScan 데이터를 처리하여 Occupancy Grid에 반영
        angle_min = scan.angle_min  # 시작 각도
        angle_increment = scan.angle_increment  # 각 스캔 사이의 각도 차이
        ranges = scan.ranges  # 스캔 데이터

        for i, distance in enumerate(ranges):
            # 스캔 거리가 유효할 때만 처리
            if scan.range_min < distance < scan.range_max:
                # 스캔 각도 계산
                scan_angle = angle_min + i * angle_increment
                total_angle = robot_yaw + scan_angle

                # 로봇의 위치를 기준으로 스캔된 점의 절대 좌표 계산
                obstacle_x = robot_x + distance * np.cos(total_angle)
                obstacle_y = robot_y + distance * np.sin(total_angle)

                # Occupancy Grid 좌표로 변환
                grid_x = int((obstacle_x - self.grid_map.info.origin.position.x) / self.grid_map.info.resolution)
                grid_y = int((obstacle_y - self.grid_map.info.origin.position.y) / self.grid_map.info.resolution)

                # 3. 맵 범위 내에 있는 경우에만 업데이트
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    index = grid_y * self.map_width + grid_x
                    self.grid_map.data[index] = 100  # 점유 영역을 100으로 설정

        # OccupancyGrid 맵을 퍼블리시
        self.map_pub.publish(self.grid_map)

if __name__ == '__main__':
    try:
        generator = GridMapGenerator()
        rospy.spin()  # ROS 노드를 실행하고 콜백 함수가 처리되도록 함
    except rospy.ROSInterruptException:
        pass

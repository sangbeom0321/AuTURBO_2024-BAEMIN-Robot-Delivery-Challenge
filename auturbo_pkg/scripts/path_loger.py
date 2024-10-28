#!/usr/bin/env python
import rospy
import math
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import os

class PathRecorder:
    def __init__(self):
        rospy.init_node('path_recorder', anonymous=True)

        # 경로를 저장할 리스트
        self.path_data = []

        # 이전 위치 저장
        self.prev_x = None
        self.prev_y = None

        # 오돔 토픽 구독
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 루프 속도 설정
        self.rate = rospy.Rate(10)  # 10Hz

        # 파일 저장 경로 설정
        self.file_path = os.path.join("/root/catkin_ws/src/auturbo_pkg/paths", "path_data.json")

    def odom_callback(self, msg):
        # 현재 위치
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # 이전 위치가 없으면 저장하지 않음
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = current_x
            self.prev_y = current_y
            return

        # 두 지점 사이의 거리 계산
        distance = math.sqrt((current_x - self.prev_x) ** 2 + (current_y - self.prev_y) ** 2)

        # 5cm(0.05m) 이상 이동했을 때만 경로에 추가
        if distance >= 0.05:
            pose_data = {
                'x': current_x,
                'y': current_y,
                'z': msg.pose.pose.position.z,
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'timestamp': rospy.Time.now().to_sec()
            }

            self.path_data.append(pose_data)

            # 이전 위치 업데이트
            self.prev_x = current_x
            self.prev_y = current_y

    def save_to_file(self):
        with open(self.file_path, 'w') as json_file:
            json.dump(self.path_data, json_file, indent=4)
        rospy.loginfo(f"Path saved to {self.file_path}")

    def run(self):
        rospy.on_shutdown(self.save_to_file)  # 종료 시 파일 저장
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        recorder = PathRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass

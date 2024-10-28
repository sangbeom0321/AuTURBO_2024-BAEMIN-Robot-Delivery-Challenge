#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import DillyCmd, DillyCmdResponse, WoowaDillyStatus, SkidSteer6wUGVCtrlCmd
from morai_msgs.srv import WoowaDillyEventCmdSrv
from datetime import datetime
from tf.transformations import euler_from_quaternion

import math

class StateMachineGoalPublisher:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('state_machine_goal_publisher_node')

        # 상태 정의
        self.MOVING = 0
        self.WAIT_PICKUP = 1
        self.WAIT_PICKDOWN = 2
        self.RECOVERY = 3
        self.BRUTE_MOVE = 4
        self.STOP = 5
        self.JAM = 6
        self.state = self.MOVING
        self.last_state = self.MOVING
        self.tasks = []

        self.goal_indoor_points = []
        self.goal_outdoor_points = []
        self.traffic_zones = []
        self.last_goal = [0.0, 0.0, 0.0]

        # 현재 위치와 경로 데이터
        self.current_position = None
        self.current_orientation = None
        self.path = None
        self.cmd_msg = None
        self.last_global_goal_pub_time = None
        self.last_teleport_time = None
        self.empty_map = None
        self.last_jammed_time = 0.0
        self.last_jammed_position = [0.0, 0.0, 0.0]
        
        # txt 파일에서 목표 지점 읽기
        self.read_goal_points(self.goal_indoor_points, "/root/catkin_ws/src/auturbo_pkg/tasks_and_goals/goal_pose_in.txt")
        self.read_goal_points(self.goal_outdoor_points, "/root/catkin_ws/src/auturbo_pkg/tasks_and_goals/goal_pose_out.txt")
        self.read_goal_points(self.traffic_zones, "/root/catkin_ws/src/auturbo_pkg/tasks_and_goals/traffic_area.txt")

        self.current_goal = self.goal_indoor_points[4]
        self.indoor_teleport_point = self.goal_indoor_points[6]
        self.outdoor_teleport_point = self.goal_outdoor_points[6]
        self.outdoor_human_position_task_4 = self.goal_outdoor_points[7]
        self.outdoor_human_position_task_2 = self.goal_outdoor_points[8]

        self.read_tasks(self.tasks, "/root/catkin_ws/src/auturbo_pkg/tasks_and_goals/tasks.txt")
        self.current_task_index = 0
        self.goal_flag = False
        self.recovery_flag = False
        self.jam_flag = False
        print(self.tasks)

        # odom 및 path 구독, goal 퍼블리셔 설정
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        self.dillystatus_sub = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, self.dillystatus_callback)
        self.cmd_vel_sub = rospy.Subscriber('/Controller', SkidSteer6wUGVCtrlCmd, self.cmd_callback)
        self.local_costmap_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.map_callback, queue_size=10)

        # publish
        self.service_response_pub = rospy.Publisher('/DeliveryResponse', Bool, queue_size=10)
        self.Dillystop_pub = rospy.Publisher('/DillyDilveryStop', Bool, queue_size=10)
        self.local_goal_pub = rospy.Publisher('/txt_data', Float32MultiArray, queue_size=10)
        self.global_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=1)
        self.local_costmap_pub = rospy.Publisher("/move_base/local_costmap/costmap_modified", OccupancyGrid, queue_size=10)
        
        self.request = DillyCmd()
        self.response = DillyCmdResponse()

        rospy.Timer(rospy.Duration(1.0), self.path_goal_publisher)
        rospy.Timer(rospy.Duration(0.5), self.state_machine)
    
    def dillystatus_callback(self, msg):
        _ = 1

    def read_goal_points(self, goal_points, file_path):
        """ 목표 지점을 txt 파일에서 읽어옴 """
        with open(file_path, 'r') as file:
            for line in file:
                x, y, z = map(float, line.strip().split())
                goal_points.append([x, y, z])

    def read_tasks(self, tasks, file_path):
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.split()  # 공백 기준으로 분리
                tasks.append([int(parts[0]), int(parts[1]), int(parts[2]), str(parts[3])])

    def odom_callback(self, msg):
        # 현재 위치 저장
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.current_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def path_callback(self, path_msg):
        # path 데이터 저장
        self.path = path_msg.poses

    def get_distance(self, position1, position2):
        """ 두 좌표 사이의 유클리드 거리를 계산하는 함수 """
        return math.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)
    
    def map_callback(self, msg):
        if self.empty_map == None:
            self.empty_map = msg
            self.empty_map.data = [0] * len(msg.data)
        
        if self.is_traffic_zone() or (self.current_task[3] == 'i' and self.current_task[2] == 6):
            self.local_costmap_pub.publish(self.empty_map)
        else:
            self.local_costmap_pub.publish(msg)
    
    def is_traffic_zone(self):
        if self.traffic_zones[0][0] <= self.current_position[0] <= self.traffic_zones[1][0] and self.traffic_zones[0][1] <= self.current_position[1] <= self.traffic_zones[1][1]:
            return True
        
        elif self.traffic_zones[2][0] <= self.current_position[0] <= self.traffic_zones[3][0] and self.traffic_zones[2][1] <= self.current_position[1] <= self.traffic_zones[3][1]:
            return True
        
        elif self.traffic_zones[4][0] <= self.current_position[0] <= self.traffic_zones[5][0] and self.traffic_zones[4][1] <= self.current_position[1] <= self.traffic_zones[5][1]:
            return True
        
        else:
            return False
    
    def get_orientation_gap(self, position1, position2):
        delta_x = position2[0] - position1[0]
        delta_y = position2[1] - position1[1]

        target_angle = math.atan2(delta_y, delta_x)

        if target_angle < 0:
            target_angle += 2 * math.pi

        quaternion = self.current_orientation
        roll, pitch, yaw = euler_from_quaternion(quaternion)  # yaw가 현재 로봇의 방향

        if yaw < 0:
            yaw += 2 * math.pi

        angle_gap = target_angle - yaw
        angle_gap = angle_gap % (2 * math.pi)

        if angle_gap > 0:
            rotation_direction = True
        else:
            rotation_direction = False

        return angle_gap, rotation_direction

    def path_goal_publisher(self, event):

        if self.current_position is None or self.path is None or len(self.path) == 0:
            return

        goal_pose = Float32MultiArray()
        found_goal = False

        # 현재 위치에서 경로 상의 점들과의 거리를 계산하여 5미터 떨어진 지점 찾기
        for pose_stamped in self.path:
            path_position = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
            distance = self.get_distance(self.current_position, path_position)

            # 경로 상에서 현재 위치로부터 5미터 떨어진 첫 번째 지점을 찾음
            if distance >= 5.0:
                goal_pose.data = path_position
                found_goal = True
                break

        # 5미터 떨어진 지점을 찾지 못했을 경우 마지막 지점을 goal로 설정
        if not found_goal:
            last_pose = self.path[-1].pose.position
            goal_pose.data = [last_pose.x, last_pose.y, 0.0]

        # 목표 지점을 퍼블리시
        rospy.loginfo(f"Current Local Goal: [{goal_pose.data[0]},{goal_pose.data[1]}]")
        # rospy.loginfo(f"Current Local Goal: [{goal_pose.data}]")
        self.local_goal_pub.publish(goal_pose)
    
    def cmd_callback(self, msg):
        self.cmd_msg = msg
        if not (self.state == self.RECOVERY or self.state == self.BRUTE_MOVE or self.state == self.STOP or self.state == self.JAM):
            self.cmd_vel_pub.publish(self.cmd_msg)

    def manual_movement_front(self):
        msg = SkidSteer6wUGVCtrlCmd()
        msg.cmd_type = 3
        msg.Target_linear_velocity = 2.0
        msg.Target_angular_velocity = 0.0
        self.cmd_vel_pub.publish(msg)

    def manual_movement_back(self):
        msg = SkidSteer6wUGVCtrlCmd()
        msg.cmd_type = 3
        msg.Target_linear_velocity = -1.0
        msg.Target_angular_velocity = 0.0
        self.cmd_vel_pub.publish(msg)

    def manual_movement_fast_rotation(self, direction):
        msg = SkidSteer6wUGVCtrlCmd()
        msg.cmd_type = 3
        msg.Target_linear_velocity = 0.0
        if direction == True:
            msg.Target_angular_velocity = -0.2
        else:
            msg.Target_angular_velocity = 0.2

        self.cmd_vel_pub.publish(msg)

    def manual_movement_slow_rotation(self):
        msg = SkidSteer6wUGVCtrlCmd()
        msg.cmd_type = 3
        msg.Target_linear_velocity = 0.0
        msg.Target_angular_velocity = 0.2
        self.cmd_vel_pub.publish(msg)

    def manual_movement_stop(self):
        msg = SkidSteer6wUGVCtrlCmd()
        msg.cmd_type = 3
        msg.Target_linear_velocity = 0.0
        msg.Target_angular_velocity = 0.0
        self.cmd_vel_pub.publish(msg)

    def state_machine(self, event):
        """ 상태 기계 """
        self.current_task = self.tasks[self.current_task_index]
        if self.get_distance(self.current_position, self.current_goal) < 3.0:
            self.goal_flag = True

        if int(datetime.now().timestamp()) - self.last_jammed_time > 4:
            if self.get_distance(self.current_position, self.last_jammed_position) < 0.3 and not self.state == self.BRUTE_MOVE and not self.jam_flag:
                self.last_state = self.state
                self.jam_flag = True
                self.state = self.JAM
            else: 
                self.jam_flag = False
            self.last_jammed_time = int(datetime.now().timestamp())
            self.last_jammed_position = self.current_position

        if self.state == self.MOVING:
            # 목표 지점 설정
            if self.current_position[0] > 170 and self.current_task[3] ==  'i' and not self.goal_flag:
                self.state = self.RECOVERY
            elif self.current_position[0] < 170 and self.current_task[3] ==  'o' and not self.goal_flag:
                self.state = self.RECOVERY

            else:
                if self.current_task[3] == 'i':
                    self.current_goal = self.goal_indoor_points[self.current_task[2]]
                elif self.current_task[3] == 'o':
                    self.current_goal = self.goal_outdoor_points[self.current_task[2]]

            # 이 부분이 도착했다고 판정하는 부분인데 검증이 필요함
            if self.goal_flag:
                # 인도어 태스크인데, 텔레포트를 타는 태스크가 아니라면? 픽업을 하는 태스크임
                if self.current_task[3] == 'i' and not self.current_task[2] == 6: 
                    self.state = self.WAIT_PICKUP
                    self.goal_flag = False

                elif self.current_task[3] == 'o' and not self.current_task[2] == 6:
                    self.state = self.WAIT_PICKDOWN
                    self.goal_flag = False
                
                # 도착했다고 판정된 뒤에 텔레포트를 탔다면 아래 조건을 만족할 것
                elif self.current_position[0] > 170 and self.current_task[3] ==  'i': 
                    self.current_task_index += 1
                    self.goal_flag = False
                    self.last_teleport_time = int(datetime.now().timestamp())
                    self.state = self.STOP

                elif self.current_position[0] < 170 and self.current_task[3] ==  'o':
                    self.current_task_index += 1
                    self.goal_flag = False
                    self.last_teleport_time = int(datetime.now().timestamp())
                    self.state = self.STOP

        elif self.state == self.WAIT_PICKUP:
            if self.get_distance(self.current_position, self.current_goal) < 1.5 :
                # self.current_goal = self.current_position
                self.pickup_item(self.current_task[2])
                if self.response and self.response.response.result:
                    rospy.loginfo(f"Goal Flag {self.goal_flag}")

                    # 서비스 응답이 성공적일 경우
                    self.state = self.MOVING
                    self.current_task_index += 1
                    self.current_task = self.tasks[self.current_task_index]
                    if self.current_task[3] == 'i':
                        self.current_goal = self.goal_indoor_points[self.current_task[2]]
                    elif self.current_task[3] == 'o':
                        self.current_goal = self.goal_outdoor_points[self.current_task[2]]
                    self.goal_flag = False

        elif self.state == self.WAIT_PICKDOWN:
            if self.get_distance(self.current_position, self.current_goal) < 1.5 :
                # self.current_goal = self.current_position
                self.pickdown_item(self.current_task[2])
                if self.response and self.response.response.result:

                    # 서비스 응답이 성공적일 경우
                    self.state = self.MOVING
                    self.current_task_index += 1
                    self.current_task = self.tasks[self.current_task_index]
                    if self.current_task[3] == 'i':
                        self.current_goal = self.goal_indoor_points[self.current_task[2]]
                    elif self.current_task[3] == 'o':
                        self.current_goal = self.goal_outdoor_points[self.current_task[2]]
                    self.goal_flag = False
                    if self.current_task_index == 5 or self.current_task_index == 11:
                        self.state = self.BRUTE_MOVE

        elif self.state == self.RECOVERY:
            if self.current_task[3] == 'i':
                if self.get_distance(self.current_position, self.outdoor_teleport_point) < 1.2 and not self.recovery_flag:
                    self.manual_movement_front()
                else:
                    self.manual_movement_back()
                    self.recovery_flag = False
                    if self.current_position[0] < 170:
                        self.manual_movement_front()
                        self.last_teleport_time = int(datetime.now().timestamp())
                        self.state = self.STOP
                        
            elif self.current_task[3] == 'o':
                if self.get_distance(self.current_position, self.indoor_teleport_point) < 1.2 and not self.recovery_flag:
                    self.manual_movement_front()
                else:
                    self.manual_movement_back()
                    self.recovery_flag = False                    
                    if self.current_position[0] > 170:
                        self.last_teleport_time = int(datetime.now().timestamp())
                        self.state = self.STOP

        elif self.state == self.STOP:
            if int(datetime.now().timestamp()) - self.last_teleport_time < 3:
                self.manual_movement_stop()
            else:
                self.state = self.MOVING

        # 오차가 많을 때 패스트 로테이션으로 맞추고 조건은 0.5, 0.5보다 오차가 적어지면 느린 회전 
        elif self.state == self.BRUTE_MOVE:

            if self.current_position[0] > 400:
                self.state = self.MOVING

            if self.current_task_index == 5:
                angle, direction = self.get_orientation_gap(self.current_position, self.outdoor_human_position_task_4)
                if  angle > 0.3:
                    self.manual_movement_fast_rotation(True)
                else:
                    self.manual_movement_front()
        
            if self.current_task_index == 11:
                angle, direction = self.get_orientation_gap(self.current_position, self.outdoor_human_position_task_2)
                if angle > 0.3:
                    self.manual_movement_fast_rotation(False)
                else:
                    self.manual_movement_front()

        elif self.state == self.JAM:
            if self.jam_flag:
                self.manual_movement_back()
            else:
                self.state = self.last_state

        if self.get_distance(self.last_goal, self.current_goal) > 1.5 or (abs(int(datetime.now().timestamp()) - self.last_global_goal_pub_time) > 5):
            self.publish_goal()
        self.last_goal = self.current_goal

        rospy.loginfo(f"Task Index: {self.current_task_index}, Current State: {self.state}, Current Global Goal: [{self.current_goal[0]},{self.current_goal[1]}], Distance: {self.get_distance(self.current_position, self.current_goal)}, Goal Flag {self.goal_flag}")

    def publish_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"

        goal_msg.pose.position.x = float(self.current_goal[0])
        goal_msg.pose.position.y = float(self.current_goal[1])
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.w = 1.0
        
        self.global_goal_pub.publish(goal_msg)
        self.last_global_goal_pub_time = int(datetime.now().timestamp())

    def pickup_item(self, index):
        rospy.wait_for_service('/WoowaDillyEventCmd')
        try:
            woowa_dilly_event_cmd = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)
            self.request.isPickup = True
            self.request.deliveryItemIndex = index
            self.response = woowa_dilly_event_cmd(self.request)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

    def pickdown_item(self, index):
        rospy.wait_for_service('/WoowaDillyEventCmd')
        try:
            woowa_dilly_event_cmd = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)
            self.request.isPickup = False
            self.request.deliveryItemIndex = index
            self.response = woowa_dilly_event_cmd(self.request)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

    def run(self):
        """ 주기적으로 상태 머신 실행 """
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        sm_goal_publisher = StateMachineGoalPublisher()
        sm_goal_publisher.run()
    except rospy.ROSInterruptException:
        pass

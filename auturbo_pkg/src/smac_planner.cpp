#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <mutex>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>

class SmacPlanner
{
public:
    SmacPlanner(ros::NodeHandle& nh)
    {
        // 토픽 구독 설정
        map_sub_ = nh.subscribe("map", 10, &SmacPlanner::mapCallback, this);
        goal_sub_ = nh.subscribe("goal_pose", 10, &SmacPlanner::goalPoseCallback, this);
        odom_sub_ = nh.subscribe("odom", 10, &SmacPlanner::odomCallback, this);

        // 경로 퍼블리셔 설정
        path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 1);
    }

    // 주 루프에서 호출될 함수
    void spin()
    {
        ros::Rate rate(10);  // 10Hz 실행
        while (ros::ok())
        {
            ros::spinOnce();  // 콜백 함수 실행

            if (current_map_ && goal_pose_ && current_odom_)  // 모든 데이터가 있을 경우에만 경로 계획 실행
            {
                std::lock_guard<std::mutex> lock(data_mutex_);  // 데이터 동기화

                // 현재 위치 및 목표 위치 가져오기
                geometry_msgs::Pose current_pose = current_odom_->pose.pose;
                geometry_msgs::Pose target_pose = goal_pose_->pose;

                // 경로 생성
                nav_msgs::Path path = planPath(current_map_, current_pose, target_pose);

                // 경로 퍼블리시
                path_pub_.publish(path);
            }

            rate.sleep();
        }
    }

private:
    // A* 노드 구조체
    struct Node
    {
        int x, y;
        double cost, heuristic;
        Node* parent;

        Node(int x, int y, double cost, double heuristic, Node* parent = nullptr)
            : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

        // 우선순위 큐에서의 비교를 위한 연산자 오버로딩
        bool operator>(const Node& other) const
        {
            return (cost + heuristic) > (other.cost + other.heuristic);
        }
    };

    // 맵, 목표 포즈, 현재 odom을 위한 콜백 함수
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_map_ = msg;
    }

    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_pose_ = msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_odom_ = msg;
    }

    // A* 경로 계획 함수
    nav_msgs::Path planPath(
        const nav_msgs::OccupancyGrid::ConstPtr& map,
        const geometry_msgs::Pose& current_odom_pose,
        const geometry_msgs::Pose& goal_pose,
        float tolerance = 0.25)
    {
        // 맵 정보 가져오기
        int width = map->info.width;
        int height = map->info.height;
        double resolution = map->info.resolution;
        std::vector<int8_t> data = map->data;

        // 시작점과 목표점을 맵 인덱스로 변환
        int start_x = (current_odom_pose.position.x - map->info.origin.position.x) / resolution;
        int start_y = (current_odom_pose.position.y - map->info.origin.position.y) / resolution;
        int goal_x = (goal_pose.position.x - map->info.origin.position.x) / resolution;
        int goal_y = (goal_pose.position.y - map->info.origin.position.y) / resolution;

        // A* 알고리즘을 위한 우선순위 큐
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
        std::vector<std::vector<bool>> closed_list(width, std::vector<bool>(height, false));

        // 시작 노드 설정
        Node* start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push(*start_node);

        // 경로 저장을 위한 노드 포인터
        Node* goal_node = nullptr;

        // 방향 벡터 (상, 하, 좌, 우, 대각선)
        std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0},
                                                       {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        // A* 알고리즘 실행
        while (!open_list.empty())
        {
            Node current = open_list.top();
            open_list.pop();

            // 목표 도달 시 루프 종료
            if (std::abs(current.x - goal_x) <= tolerance && std::abs(current.y - goal_y) <= tolerance)
            {
                goal_node = new Node(current.x, current.y, current.cost, 0, current.parent);
                break;
            }

            // 이미 방문한 노드라면 무시
            if (closed_list[current.x][current.y])
                continue;

            // 현재 노드 방문 처리
            closed_list[current.x][current.y] = true;

            // 인접한 노드 확장
            for (const auto& direction : directions)
            {
                int new_x = current.x + direction.first;
                int new_y = current.y + direction.second;

                // 맵 경계를 벗어나면 무시
                if (new_x < 0 || new_y < 0 || new_x >= width || new_y >= height)
                    continue;

                // 장애물(맵 데이터가 100인 경우) 무시
                if (data[new_x + new_y * width] == 100)
                    continue;

                // 새로운 노드 생성 및 큐에 삽입
                double new_cost = current.cost + std::hypot(direction.first, direction.second);
                Node* neighbor = new Node(new_x, new_y, new_cost, heuristic(new_x, new_y, goal_x, goal_y), new Node(current));
                open_list.push(*neighbor);
            }
        }

        // 경로 생성
        nav_msgs::Path path;
        path.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";

        if (goal_node)
        {
            Node* current = goal_node;
            while (current != nullptr)
            {
                pose.pose.position.x = current->x * resolution + map->info.origin.position.x;
                pose.pose.position.y = current->y * resolution + map->info.origin.position.y;
                path.poses.push_back(pose);
                current = current->parent;
            }
        }

        // 경로 역순으로 정렬 (start -> goal 순)
        std::reverse(path.poses.begin(), path.poses.end());

        return path;
    }

    // 휴리스틱 함수 (맨해튼 거리)
    double heuristic(int x1, int y1, int x2, int y2)
    {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);

        return D2 * std::min(dx, dy) + D * (std::max(dx, dy) - std::min(dx, dy));
    }

    // ROS 관련 멤버 변수
    ros::Subscriber map_sub_, goal_sub_, odom_sub_;
    ros::Publisher path_pub_;

    // 데이터 저장을 위한 멤버 변수
    nav_msgs::OccupancyGrid::ConstPtr current_map_;
    geometry_msgs::PoseStamped::ConstPtr goal_pose_;
    nav_msgs::Odometry::ConstPtr current_odom_;

    // 데이터 동기화를 위한 mutex
    std::mutex data_mutex_;

    double D = 1.0;         // 직선 이동 비용
    double D2 = std::sqrt(2) * 0.95;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smac_planner_node");
    ros::NodeHandle nh;

    SmacPlanner planner(nh);  // SmacPlanner 객체 생성
    planner.spin();  // 경로 계획 실행

    return 0;
}

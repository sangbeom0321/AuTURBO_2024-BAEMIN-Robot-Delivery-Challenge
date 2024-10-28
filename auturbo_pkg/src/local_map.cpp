#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <vector>
#include <algorithm>

class LocalCostmapNode {
public:
    LocalCostmapNode() {
        // Subscribers
        map_sub_ = nh_.subscribe("/map", 1, &LocalCostmapNode::mapCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &LocalCostmapNode::odomCallback, this);
        scan_sub_ = nh_.subscribe("/scan_hy", 1, &LocalCostmapNode::scanCallback, this);

        // Publisher
        local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);

        // Timer to update local costmap periodically
        update_timer_ = nh_.createTimer(ros::Duration(0.1), &LocalCostmapNode::updateLocalCostmap, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // Store the global cost map
        global_map_ = *msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store the odometry information
        odom_ = *msg;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Store the laser scan data
        laser_scan_ = *msg;
    }

    void updateLocalCostmap(const ros::TimerEvent&) {
        
        if (!global_map_.data.empty() && !laser_scan_.ranges.empty()) {
            // Create a local cost map based on global map, odometry, and laser scan
            nav_msgs::OccupancyGrid local_costmap;
            local_costmap.header.stamp = ros::Time::now();
            local_costmap.header.frame_id = "velodyne";
            local_costmap.info.resolution = global_map_.info.resolution;
            local_costmap.info.width = 50;  // Example width
            local_costmap.info.height = 50; // Example height
            std::cout << odom_.pose.pose.position.x << std::endl;
            std::cout << odom_.pose.pose.position.y << std::endl;
            local_costmap.info.origin.position.x = odom_.pose.pose.position.x;
            local_costmap.info.origin.position.y = odom_.pose.pose.position.y;
            local_costmap.info.origin.orientation.w = 1.0;

            // Fill the cost map with some values (improving based on laser scan data)
            local_costmap.data.resize(local_costmap.info.width * local_costmap.info.height, 0);
            for (size_t i = 0; i < laser_scan_.ranges.size(); ++i) {
                if (laser_scan_.ranges[i] < laser_scan_.range_max) {
                    double angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
                    double x = odom_.pose.pose.position.x + laser_scan_.ranges[i] * cos(angle);
                    double y = odom_.pose.pose.position.y + laser_scan_.ranges[i] * sin(angle);
                    int grid_x = static_cast<int>((x - local_costmap.info.origin.position.x) / local_costmap.info.resolution);
                    int grid_y = static_cast<int>((y - local_costmap.info.origin.position.y) / local_costmap.info.resolution);
                    if (grid_x >= 0 && grid_x < local_costmap.info.width && grid_y >= 0 && grid_y < local_costmap.info.height) {
                        local_costmap.data[grid_y * local_costmap.info.width + grid_x] = 100;  // Mark as an obstacle
                    }
                }
            }
            // Publish the local cost map
            local_costmap_pub_.publish(local_costmap);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher local_costmap_pub_;
    ros::Timer update_timer_;

    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::Odometry odom_;
    sensor_msgs::LaserScan laser_scan_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_costmap_node");
    LocalCostmapNode local_costmap_node;
    ros::spin();
    return 0;
}

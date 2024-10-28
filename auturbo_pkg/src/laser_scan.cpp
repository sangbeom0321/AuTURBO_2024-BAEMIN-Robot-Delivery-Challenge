#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <limits>
#include <cmath>

class PCDPublisher {
public:
  PCDPublisher() {
    ros::NodeHandle nh;
    nh.param<std::string>("pcd_topic", pcd_topic, "/lidar3D");
    nh.param<std::string>("scan_topic", scan_topic, "/scan_hy");
    nh.param<std::string>("imu_topic", imu_topic, "/imu");
    nh.param<float>("clipping_minz", clipping_minz, -0.4);
    nh.param<float>("clipping_maxz", clipping_maxz, 0.2);

    publisher_scan = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 10);
    lidar_subscription_ = nh.subscribe(pcd_topic, 10, &PCDPublisher::publishPointCloud, this);
    imu_subscriber_ = nh.subscribe(imu_topic, 10, &PCDPublisher::imuCallback, this);
  }

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_data_queue_.push_back(*msg);
  }

  void publishPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_data_queue_.empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    Eigen::Matrix4f transform = get_tf_from_imu();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(transformed_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz, clipping_maxz);
    pass.filter(*cloud_filtered);

    sensor_msgs::LaserScan laser_scan = convertPointCloudToLaserScan(cloud_filtered);
    publisher_scan.publish(laser_scan);
  }

    sensor_msgs::LaserScan convertPointCloudToLaserScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "velodyne";
    scan.header.stamp = ros::Time::now();
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 449;
    scan.range_min = 1.0;
    scan.range_max = 200.0;

    uint32_t ranges_size = static_cast<uint32_t>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.assign(ranges_size, 200.0);

    for (const auto& point : cloud->points) {
      float range = std::sqrt(point.x * point.x + point.y * point.y);
      if (range < scan.range_min || range > scan.range_max) continue;
      float angle = std::atan2(point.y, point.x);
      if (angle < scan.angle_min || angle > scan.angle_max) continue;
      int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
      if (index >= 0 && index < ranges_size) {
        scan.ranges[index] = std::min(range, scan.ranges[index]);
      }
    }
    return scan;
}

  Eigen::Matrix4f get_tf_from_imu() {
    sensor_msgs::Imu imu_data = imu_data_queue_.back();
    imu_data_queue_.clear();

    float sinr_cosp = 2 * (imu_data.orientation.w * imu_data.orientation.x + imu_data.orientation.y * imu_data.orientation.z);
    float cosr_cosp = 1 - 2 * (imu_data.orientation.x * imu_data.orientation.x + imu_data.orientation.y * imu_data.orientation.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (imu_data.orientation.w * imu_data.orientation.y - imu_data.orientation.z * imu_data.orientation.x);
    pitch = std::asin(sinp);

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation_matrix;
    return transform;
  }

  ros::Publisher publisher_scan;
  ros::Subscriber lidar_subscription_;
  ros::Subscriber imu_subscriber_;
  std::deque<sensor_msgs::Imu> imu_data_queue_;
  std::mutex mutex_;
  float clipping_minz, clipping_maxz;
  std::string pcd_topic, scan_topic, imu_topic;
  float roll, pitch;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scan_node");
  PCDPublisher node;
  ros::spin();
  return 0;
}

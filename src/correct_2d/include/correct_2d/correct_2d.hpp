#ifndef CORRECT_2D__CORRECT_2D_HPP_
#define CORRECT_2D__CORRECT_2D_HPP_

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// pcl header file
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include <pcl/filters/voxel_grid.h>  // 添加此行 filter points

namespace correct_2d
{
class Correct2d : public rclcpp::Node
{
public:
    Correct2d();
    ~Correct2d();

private:
    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    nav_msgs::msg::OccupancyGrid map_;
    sensor_msgs::msg::LaserScan scan_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ICP
    pcl::PointCloud<pcl::PointXYZ> map_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ> scan_pointcloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    void ConvertMapToPointCloud(const nav_msgs::msg::OccupancyGrid& map, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void ConvertScanToPointCloud(const sensor_msgs::msg::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void ComputeICP(const pcl::PointCloud<pcl::PointXYZ>& cloud_map, const pcl::PointCloud<pcl::PointXYZ>& cloud_scan, 
        double& x, double& y, double& yaw_rad, double& yaw_deg);




};
}  // namespace correct_2d

#endif  // CORRECT_2D__CORRECT_2D_HPP_

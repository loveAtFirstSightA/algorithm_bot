#ifndef CORRECT_2D_NDT__CORRECT_2D_NDT_HPP_
#define CORRECT_2D_NDT__CORRECT_2D_NDT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


namespace correct_2d_ndt
{
class Correct2dNDT : public rclcpp::Node
{
public:
    Correct2dNDT();
    ~Correct2dNDT();

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
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    void ConvertMapToPointCloud(const nav_msgs::msg::OccupancyGrid& map, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void ConvertScanToPointCloud(const sensor_msgs::msg::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& cloud);
    void ComputeNDT(const pcl::PointCloud<pcl::PointXYZ>& cloud_map, const pcl::PointCloud<pcl::PointXYZ>& cloud_scan, 
        double& x, double& y, double& yaw_rad, double& yaw_deg);


};

}  // namespace correct_2d_ndt

#endif  // CORRECT_2D_NDT_HPP

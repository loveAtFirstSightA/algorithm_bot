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

    // NDT



};
}  // namespace correct_2d_ndt
#endif  // CORRECT_2D_NDT__CORRECT_2D_NDT_HPP_

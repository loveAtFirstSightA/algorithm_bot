#include "correct_2d_ndt/correct_2d_ndt.hpp"

namespace correct_2d_ndt
{
Correct2dNDT::Correct2dNDT() : Node("correct_2d_ndt")
{
    spdlog::info("Correct launch");

    // 创建订阅者
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&Correct2dNDT::MapCallback, this, std::placeholders::_1));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Correct2dNDT::ScanCallback, this, std::placeholders::_1));

    // 初始化tf2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Correct2dNDT::~Correct2dNDT() {}

void Correct2dNDT::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
}

void Correct2dNDT::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = *msg;
}




}  // namespace correct_2d_ndt

#include "correct_2d_icp/correct_2d_icp.hpp"

namespace correct_2d_icp
{
Correct2dICP::Correct2dICP() : Node("correct_2d_icp")
{
    spdlog::info("Correct launch");

    // 创建订阅者
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&Correct2dICP::MapCallback, this, std::placeholders::_1));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Correct2dICP::ScanCallback, this, std::placeholders::_1));

    // 初始化tf2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Correct2dICP::~Correct2dICP() {}

void Correct2dICP::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
}

void Correct2dICP::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 更新扫描数据
    scan_ = *msg;

    // 将LaserScan数据转换为PointCloud
    ConvertScanToPointCloud(scan_, scan_pointcloud_);

    // 假设你已经通过其他方法生成了地图点云
    ConvertMapToPointCloud(map_, map_pointcloud_);

    // 定义 ICP 结果变量
    double x, y, yaw_rad, yaw_deg;

    if (!tf_buffer_->canTransform("map", "base_scan", tf2::TimePointZero)) {
        spdlog::warn("No transform available from scan to map.");
        return;
    }

    // 使用ICP计算地图和扫描点云之间的坐标关系
    ComputeICP(map_pointcloud_, scan_pointcloud_, x, y, yaw_rad, yaw_deg);

    // 输出计算结果
    spdlog::info("ICP Transformation - X: {:.4f}, Y: {:.4f}, Yaw (rad): {:.4f}, Yaw (deg): {:.4f}", x, y, yaw_rad, yaw_deg);
    spdlog::info("");
}

// ICP
void Correct2dICP::ComputeICP(const pcl::PointCloud<pcl::PointXYZ>& cloud_map, const pcl::PointCloud<pcl::PointXYZ>& cloud_scan, 
    double& x, double& y, double& yaw_rad, double& yaw_deg)
{
    // 创建 ICP 对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // 设置输入点云和目标点云
    icp.setInputSource(cloud_scan.makeShared());
    icp.setInputTarget(cloud_map.makeShared());

    // 设置 ICP 参数
    icp.setMaxCorrespondenceDistance(0.5);  // 增加最大对应点的距离
    icp.setTransformationEpsilon(1e-6);    // 放宽变换精度
    icp.setEuclideanFitnessEpsilon(1e-5);  // 放宽匹配的容忍度
    icp.setMaximumIterations(1000);        // 最大迭代次数

    // 可选：应用滤波器来减少点云数据量，提高计算效率
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_scan.makeShared());
    voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素大小与地图分辨率相同
    pcl::PointCloud<pcl::PointXYZ> cloud_scan_filtered;
    voxel_grid.filter(cloud_scan_filtered);

    // 执行 ICP 配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    // 检查是否收敛
    if (icp.hasConverged()) {
        spdlog::info("ICP converged, score: {}", icp.getFitnessScore());
    } else {
        spdlog::warn("ICP did not converge. Score: {}", icp.getFitnessScore());
        return;
    }

    // 获取变换矩阵
    Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

    // 从变换矩阵中提取平移 (x, y)
    x = transformation_matrix(0, 3);
    y = transformation_matrix(1, 3);

    // 从变换矩阵中提取旋转矩阵，计算 yaw（弧度）
    Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
    float yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    
    // 转换为度
    yaw_rad = yaw;
    yaw_deg = yaw * 180.0 / M_PI;
    
    // 输出日志
    spdlog::info("ICP Transformation - X: {:.2f}, Y: {:.2f}, Yaw (rad): {:.2f}, Yaw (deg): {:.2f}", x, y, yaw_rad, yaw_deg);
    spdlog::info("");
}

void Correct2dICP::ConvertMapToPointCloud(const nav_msgs::msg::OccupancyGrid& map, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();
    for (size_t i = 0; i < map.data.size(); i++) {
        int x = i % map.info.width;
        int y = i / map.info.width;

        if (map.data[i] == 100) {
            double world_x = x * map.info.resolution + map.info.origin.position.x;
            double world_y = y * map.info.resolution + map.info.origin.position.y;
            double world_z = 0.0; // 假设地图在2D平面上，z坐标为0

            pcl::PointXYZ point;
            point.x = world_x;
            point.y = world_y;
            point.z = world_z;

            cloud.points.push_back(point);
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
}

void Correct2dICP::ConvertScanToPointCloud(const sensor_msgs::msg::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& cloud) 
{
    // 在查找变换之前，检查 "map" 和 "scan" 坐标系的变换是否可用
    try {
        if (tf_buffer_->canTransform("map", "base_scan", tf2::TimePointZero)) {
            // 查找扫描到地图坐标系的变换
            geometry_msgs::msg::TransformStamped tf;
            tf = tf_buffer_->lookupTransform("map", "base_scan", tf2::TimePointZero);

            // 获取扫描数据中的参数
            const float angle_min = scan.angle_min;
            const float angle_increment = scan.angle_increment;
            const float range_min = scan.range_min;
            const float range_max = scan.range_max;

            cloud.clear();  // 清空PointCloud数据

            // 遍历激光数据，逐点转换到地图坐标系
            for (size_t i = 0; i < scan.ranges.size(); ++i) {
                // 跳过无效数据
                if (scan.ranges[i] < range_min || scan.ranges[i] > range_max) {
                    continue;
                }

                // 计算当前点的角度
                const float angle = angle_min + i * angle_increment;

                // 将极坐标转换为笛卡尔坐标
                const float x = scan.ranges[i] * cos(angle);
                const float y = scan.ranges[i] * sin(angle);

                // 使用tf2转换点坐标到地图坐标系
                geometry_msgs::msg::PointStamped laser_point;
                laser_point.header.frame_id = "scan";  // 设置激光坐标系

                // 使用 rclcpp::Clock 获取当前时间戳并转换为 Time 类型
                rclcpp::Clock clock;
                laser_point.header.stamp = clock.now(); // 获取当前时间戳并设置

                laser_point.point.x = x;
                laser_point.point.y = y;
                laser_point.point.z = 0.0;

                // 变换点到map坐标系
                geometry_msgs::msg::PointStamped map_point;
                try {
                    tf2::doTransform(laser_point, map_point, tf);
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                    continue;
                }

                // 将变换后的点加入PointCloud
                pcl::PointXYZ pcl_point;
                pcl_point.x = map_point.point.x;
                pcl_point.y = map_point.point.y;
                pcl_point.z = map_point.point.z;
                cloud.push_back(pcl_point);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "'map' to 'scan' transform is not available yet.");
        }
    } catch (const tf2::LookupException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    }
}



}  // namespace correct_2d

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
    // 更新扫描数据
    scan_ = *msg;

    // 将 LaserScan 数据转换为 PointCloud
    ConvertScanToPointCloud(scan_, scan_pointcloud_);

    // 假设你已经通过其他方法生成了地图点云
    ConvertMapToPointCloud(map_, map_pointcloud_);

    // 定义 NDT 结果变量
    double x, y, yaw_rad, yaw_deg;

    // 检查 tf 变换是否可用
    if (!tf_buffer_->canTransform("map", "base_scan", tf2::TimePointZero)) {
        spdlog::warn("No transform available from scan to map.");
        return;
    }

    // 使用 NDT 计算地图和扫描点云之间的坐标关系
    ComputeNDT(map_pointcloud_, scan_pointcloud_, x, y, yaw_rad, yaw_deg);

    // 输出计算结果
    spdlog::info("NDT Transformation - X: {:.4f}, Y: {:.4f}, Yaw (rad): {:.4f}, Yaw (deg): {:.4f}", x, y, yaw_rad, yaw_deg);
    spdlog::info("");
}

void Correct2dNDT::ComputeNDT(const pcl::PointCloud<pcl::PointXYZ>& cloud_map, 
                              const pcl::PointCloud<pcl::PointXYZ>& cloud_scan, 
                              double& x, double& y, double& yaw_rad, double& yaw_deg) {
    // 参数优化
    ndt_.setResolution(0.5);               // 更细致的分辨率
    ndt_.setTransformationEpsilon(1e-8);  // 更高的变换精度
    ndt_.setStepSize(0.05);                // 更小的步长
    ndt_.setMaximumIterations(50);         // 增加最大迭代次数

    // 滤波与预处理
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f); // 更小体素
    voxel_grid.setInputCloud(cloud_scan.makeShared());
    voxel_grid.filter(filtered_scan);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_scan.makeShared());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(filtered_scan);

    // 设置输入点云
    ndt_.setInputSource(filtered_scan.makeShared());
    ndt_.setInputTarget(cloud_map.makeShared());

    // 提供初始估计
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    // initial_guess(0, 3) = x; // 假设x, y, yaw初值已知
    // initial_guess(1, 3) = y;
    initial_guess(0, 3) = 0.1; // 假设x, y, yaw初值已知
    initial_guess(1, 3) = 0.1;
    initial_guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // 执行 NDT
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    ndt_.align(aligned_cloud, initial_guess);

    // 检查结果
    if (!ndt_.hasConverged() || ndt_.getFitnessScore() > 0.001) {
        spdlog::warn("NDT未收敛或匹配误差过高, Score: {}", ndt_.getFitnessScore());
        return;
    }

    Eigen::Matrix4f transformation_matrix = ndt_.getFinalTransformation();
    x = transformation_matrix(0, 3);
    y = transformation_matrix(1, 3);
    yaw_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));
    yaw_deg = yaw_rad * 180.0 / M_PI;
    spdlog::info("NDT成功: X: {:.2f}, Y: {:.2f}, Yaw (rad): {:.2f}, Yaw (deg): {:.2f}", x, y, yaw_rad, yaw_deg);
}

void Correct2dNDT::ConvertMapToPointCloud(const nav_msgs::msg::OccupancyGrid& map, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();  // 清空点云

    // 遍历地图中的每个单元格
    for (size_t i = 0; i < map.data.size(); i++) {
        if (map.data[i] == 100) {  // 只处理占用单元格
            // 计算单元格的索引
            int x_index = i % map.info.width;
            int y_index = i / map.info.width;

            // 计算对应的世界坐标
            float world_x = x_index * map.info.resolution + map.info.origin.position.x;
            float world_y = y_index * map.info.resolution + map.info.origin.position.y;

            // 创建点并加入点云
            pcl::PointXYZ point;
            point.x = world_x;
            point.y = world_y;
            point.z = 0.0;  // 假设地图为2D平面，z坐标为0
            cloud.push_back(point);
        }
    }

    // 设置点云属性
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
}

void Correct2dNDT::ConvertScanToPointCloud(const sensor_msgs::msg::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud.clear();  // 清空点云

    try {
        // 检查 "map" 到 "base_scan" 的变换是否可用
        if (!tf_buffer_->canTransform("map", scan.header.frame_id, tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), "Transform from '%s' to 'map' not available.", scan.header.frame_id.c_str());
            return;
        }

        // 获取变换信息
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform("map", scan.header.frame_id, tf2::TimePointZero);

        // 遍历激光扫描数据
        const float angle_min = scan.angle_min;
        const float angle_increment = scan.angle_increment;
        const float range_min = scan.range_min;
        const float range_max = scan.range_max;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float range = scan.ranges[i];

            // 跳过无效数据
            if (range < range_min || range > range_max || std::isnan(range)) {
                continue;
            }

            // 计算激光点在局部坐标系的角度和坐标
            float angle = angle_min + i * angle_increment;
            float local_x = range * std::cos(angle);
            float local_y = range * std::sin(angle);

            // 创建局部点
            geometry_msgs::msg::PointStamped local_point;
            local_point.header.frame_id = scan.header.frame_id;
            local_point.point.x = local_x;
            local_point.point.y = local_y;
            local_point.point.z = 0.0;

            // 将局部点转换到地图坐标系
            geometry_msgs::msg::PointStamped map_point;
            tf2::doTransform(local_point, map_point, transform);

            // 将变换后的点加入点云
            pcl::PointXYZ pcl_point;
            pcl_point.x = map_point.point.x;
            pcl_point.y = map_point.point.y;
            pcl_point.z = map_point.point.z;
            cloud.push_back(pcl_point);
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform scan to map frame: %s", ex.what());
    }

    // 设置点云属性
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
}




} // namespace correct_2d_ndt

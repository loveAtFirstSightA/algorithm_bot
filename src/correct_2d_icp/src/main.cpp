#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "correct_2d_icp/correct_2d_icp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<correct_2d_icp::Correct2dICP>());
    rclcpp::shutdown();
    return 0;
}

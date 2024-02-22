#include <rclcpp/rclcpp.hpp>

#include "ugv_nav4d.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ugv_nav4d_ros2::PathPlannerNode>());
    rclcpp::shutdown();
	return 0;
}
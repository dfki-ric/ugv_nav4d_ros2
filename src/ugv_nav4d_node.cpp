#include <rclcpp/rclcpp.hpp>

#include "ugv_nav4d.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
	return 0;
}
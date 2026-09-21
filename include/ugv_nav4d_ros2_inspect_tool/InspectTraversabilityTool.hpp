#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <std_msgs/msg/string.hpp>
#include "ugv_nav4d_ros2/srv/inspect_traversability.hpp"

namespace ugv_nav4d_ros2::ugv_nav4d_ros2_inspect_tool
{
class InspectTraversabilityTool : public rviz_common::Tool
{
    Q_OBJECT
public:
    InspectTraversabilityTool();
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<ugv_nav4d_ros2::srv::InspectTraversability>::SharedPtr client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};
}

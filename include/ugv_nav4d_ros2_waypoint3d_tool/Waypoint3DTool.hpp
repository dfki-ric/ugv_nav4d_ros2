#ifndef UGV_NAV4D_ROS2_WAYPOINT3D_TOOL_HPP_
#define UGV_NAV4D_ROS2_WAYPOINT3D_TOOL_HPP_

#include <memory>

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/tool.hpp>

#include "ugv_nav4d_ros2/srv/edit_waypoint.hpp"
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace rviz_rendering
{
class Arrow;
}

namespace rviz_common
{
namespace properties
{
class StringProperty;
class IntProperty;
}
}

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_waypoint3d_tool
{

/**
 * Appends a waypoint to the planner's waypoint queue by clicking the true 3D
 * point on the rendered MLS/traversability map (drag sets the yaw). The queued
 * waypoints are planned through, in click order, when the next goal pose is
 * sent. The queue can be edited via the clear_waypoints / remove_last_waypoint
 * services and is visualized on the waypoint_markers topic.
 */
class Waypoint3DTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    Waypoint3DTool();
    ~Waypoint3DTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private Q_SLOTS:
    void updateTopic();

private:
    void publishWaypoint(double theta);

    std::shared_ptr<rviz_rendering::Arrow> arrow_;

    enum State
    {
        Position,
        Orientation
    };
    State state_;
    double angle_;
    Ogre::Vector3 waypoint_position_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Client<ugv_nav4d_ros2::srv::EditWaypoint>::SharedPtr edit_client_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    double fetched_dist_to_ground_{0.0};

    void fetchDistToGround();

    rviz_common::properties::StringProperty* topic_property_;
    rviz_common::properties::IntProperty* replace_index_property_;
};

} // namespace ugv_nav4d_ros2_waypoint3d_tool
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_WAYPOINT3D_TOOL_HPP_

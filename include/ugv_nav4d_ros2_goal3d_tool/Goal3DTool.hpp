#ifndef UGV_NAV4D_ROS2_GOAL3D_TOOL_HPP_
#define UGV_NAV4D_ROS2_GOAL3D_TOOL_HPP_

#include <memory>

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/tool.hpp>
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
class FloatProperty;
}
}

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_goal3d_tool
{

/**
 * Goal tool that picks the true 3D point on the rendered scene (e.g. the MLS or
 * traversability map patches) instead of projecting onto the z=0 ground plane
 * like the stock "2D Goal Pose" tool. Click on a patch and drag to set the yaw;
 * on release a PoseStamped with the picked z (+ optional offset) is published.
 */
class Goal3DTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    Goal3DTool();
    ~Goal3DTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private Q_SLOTS:
    void updateTopic();

private:
    void publishGoal(double theta);

    std::shared_ptr<rviz_rendering::Arrow> arrow_;

    enum State
    {
        Position,
        Orientation
    };
    State state_;
    double angle_;
    Ogre::Vector3 goal_position_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty* topic_property_;
    rviz_common::properties::FloatProperty* z_offset_property_;
};

} // namespace ugv_nav4d_ros2_goal3d_tool
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_GOAL3D_TOOL_HPP_

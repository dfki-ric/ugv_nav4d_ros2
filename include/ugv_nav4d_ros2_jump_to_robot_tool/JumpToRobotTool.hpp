#pragma once

#include <rviz_common/tool.hpp>

namespace rviz_common
{
namespace properties
{
class StringProperty;
}
}

namespace ugv_nav4d_ros2::ugv_nav4d_ros2_jump_to_robot_tool
{

/**
 * One-shot tool: clicking it in the toolbar points the current view at the
 * robot's TF frame, then hands control back to the default tool.
 */
class JumpToRobotTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    JumpToRobotTool();
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    void update(float wall_dt, float ros_dt) override;

private:
    void jumpToRobot();

    rviz_common::properties::StringProperty* frame_property_;
    bool switch_back_ = false;
};

}

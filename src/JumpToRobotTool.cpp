#include "JumpToRobotTool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <OgreQuaternion.h>
#include <OgreVector.h>

namespace ugv_nav4d_ros2::ugv_nav4d_ros2_jump_to_robot_tool
{

JumpToRobotTool::JumpToRobotTool()
{
    frame_property_ = new rviz_common::properties::StringProperty(
        "Robot frame", "arter/base_link",
        "TF frame the view jumps to when the tool is triggered.",
        getPropertyContainer());
}

void JumpToRobotTool::onInitialize()
{
    setName("Jump To Robot");
}

void JumpToRobotTool::jumpToRobot()
{
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    const std::string frame = frame_property_->getStdString();

    if (context_->getFrameManager()->getTransform(frame, position, orientation)) {
        if (auto* view = context_->getViewManager()->getCurrent()) {
            view->lookAt(position);
        }
        setStatus(QString("Centered view on '%1'.").arg(QString::fromStdString(frame)));
    } else {
        setStatus(QString("No TF for frame '%1'; view not moved.")
                      .arg(QString::fromStdString(frame)));
    }
}

void JumpToRobotTool::activate()
{
    jumpToRobot();

    // A tool stays active once selected; this one is a one-shot action, so
    // return control on the next update tick (switching inside activate()
    // would re-enter the tool manager).
    switch_back_ = true;
}

void JumpToRobotTool::deactivate()
{
}

void JumpToRobotTool::update(float /*wall_dt*/, float /*ros_dt*/)
{
    if (switch_back_) {
        switch_back_ = false;
        auto* tool_manager = context_->getToolManager();
        tool_manager->setCurrentTool(tool_manager->getDefaultTool());
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    ugv_nav4d_ros2::ugv_nav4d_ros2_jump_to_robot_tool::JumpToRobotTool,
    rviz_common::Tool)

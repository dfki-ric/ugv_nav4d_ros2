#include "Waypoint3DTool.hpp"

#include <cmath>
#include <string>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/render_window.hpp>

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_waypoint3d_tool
{

Waypoint3DTool::Waypoint3DTool()
: rviz_common::Tool()
, state_(Position)
, angle_(0.0)
, waypoint_position_(Ogre::Vector3::ZERO)
{
    shortcut_key_ = 'w';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/ugv_nav4d_ros2/add_waypoint",
        "Topic on which the picked 3D waypoint pose is published.",
        getPropertyContainer(), SLOT(updateTopic()), this);

    replace_index_property_ = new rviz_common::properties::IntProperty(
        "Replace Index", 0,
        "0: each click appends a new waypoint. N > 0: the next click replaces "
        "waypoint N (as numbered in the markers), then resets to 0.",
        getPropertyContainer());
    replace_index_property_->setMin(0);
}

Waypoint3DTool::~Waypoint3DTool() = default;

void Waypoint3DTool::onInitialize()
{
    arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 1.0f, 0.08f, 0.3f, 0.2f);
    arrow_->setColor(0.0f, 0.8f, 1.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);

    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

    setName("Add 3D Waypoint");
    updateTopic();
}

void Waypoint3DTool::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_property_->getStdString(), 1);
    edit_client_ = raw_node->create_client<ugv_nav4d_ros2::srv::EditWaypoint>(
        "/ugv_nav4d_ros2/edit_waypoint");
    clock_ = raw_node->get_clock();
}

void Waypoint3DTool::fetchDistToGround()
{
    if (!param_client_)
    {
        rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(raw_node, "/ugv_nav4d_ros2");
    }
    if (!param_client_->service_is_ready())
    {
        return;  // planner not up; keep the last known value
    }
    param_client_->get_parameters({"distToGround"},
        [this](std::shared_future<std::vector<rclcpp::Parameter>> future)
        {
            const auto params = future.get();
            if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                fetched_dist_to_ground_ = params[0].as_double();
            }
        });
}

void Waypoint3DTool::activate()
{
    fetchDistToGround();
    setStatus("Click on a map patch to append a waypoint, drag to set its orientation.");
    state_ = Position;
}

void Waypoint3DTool::deactivate()
{
    if (arrow_)
    {
        arrow_->getSceneNode()->setVisible(false);
    }
    state_ = Position;
}

int Waypoint3DTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
    int flags = 0;

    if (event.leftDown())
    {
        // Pick the true 3D point on the rendered geometry (MLS/trav patches).
        // Falls back to the z=0 ground plane when clicking into empty space.
        Ogre::Vector3 picked;
        if (context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, picked))
        {
            waypoint_position_ = picked;
        }
        else
        {
            auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);
            if (!projection.first)
            {
                return flags;
            }
            waypoint_position_ = projection.second;
        }

        arrow_->setPosition(waypoint_position_);
        state_ = Orientation;
        flags |= Render;
    }
    else if (event.type == QEvent::MouseMove && event.left() && state_ == Orientation)
    {
        // Intersect the mouse ray with the horizontal plane through the picked
        // point, so the yaw drag behaves like the 2D tool but at the patch height.
        auto camera = rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(
            event.panel->getRenderWindow());
        const Ogre::Ray mouse_ray = camera->getCameraToViewportRay(
            static_cast<float>(event.x) / static_cast<float>(event.panel->width()),
            static_cast<float>(event.y) / static_cast<float>(event.panel->height()));

        const Ogre::Plane waypoint_plane(Ogre::Vector3::UNIT_Z, waypoint_position_.z);
        const auto intersection = mouse_ray.intersects(waypoint_plane);
        if (intersection.first)
        {
            const Ogre::Vector3 current = mouse_ray.getPoint(intersection.second);
            angle_ = std::atan2(current.y - waypoint_position_.y, current.x - waypoint_position_.x);

            arrow_->getSceneNode()->setVisible(true);
            // The arrow mesh points down (-z); rotate it into the xy-plane, then yaw.
            arrow_->setOrientation(
                Ogre::Quaternion(Ogre::Radian(angle_), Ogre::Vector3::UNIT_Z) *
                Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y));
            flags |= Render;
        }
    }
    else if (event.leftUp() && state_ == Orientation)
    {
        publishWaypoint(angle_);
        arrow_->getSceneNode()->setVisible(false);
        state_ = Position;
        // Deliberately NOT Finished: the tool stays active so several waypoints
        // can be appended in a row without re-selecting it in the toolbar.
        flags |= Render;
    }

    return flags;
}

void Waypoint3DTool::publishWaypoint(double theta)
{
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = context_->getFixedFrame().toStdString();
    waypoint.header.stamp = clock_->now();

    waypoint.pose.position.x = waypoint_position_.x;
    waypoint.pose.position.y = waypoint_position_.y;
    // Picked points are on the ground surface; the planner expects body-frame
    // poses, so add the node's distToGround (fetched on tool activation).
    waypoint.pose.position.z = waypoint_position_.z + fetched_dist_to_ground_;

    waypoint.pose.orientation.x = 0.0;
    waypoint.pose.orientation.y = 0.0;
    waypoint.pose.orientation.z = std::sin(theta / 2.0);
    waypoint.pose.orientation.w = std::cos(theta / 2.0);

    const int replace_index = replace_index_property_->getInt();
    if (replace_index > 0)
    {
        if (!edit_client_->service_is_ready())
        {
            setStatus("edit_waypoint service unavailable; waypoint not replaced.");
            return;
        }
        auto request = std::make_shared<ugv_nav4d_ros2::srv::EditWaypoint::Request>();
        request->index = static_cast<uint32_t>(replace_index);
        request->remove = false;
        request->pose = waypoint.pose;
        edit_client_->async_send_request(request);
        RVIZ_COMMON_LOG_INFO_STREAM(
            "Replacing waypoint " << replace_index << " with position ("
            << waypoint.pose.position.x << ", " << waypoint.pose.position.y
            << ", " << waypoint.pose.position.z << "), yaw " << theta);
        replace_index_property_->setInt(0); // one-shot: back to append mode
        return;
    }

    RVIZ_COMMON_LOG_INFO_STREAM(
        "Publishing waypoint on " << topic_property_->getStdString()
        << ": position (" << waypoint.pose.position.x << ", " << waypoint.pose.position.y
        << ", " << waypoint.pose.position.z << "), yaw " << theta
        << ", frame " << waypoint.header.frame_id);

    publisher_->publish(waypoint);
}

} // namespace ugv_nav4d_ros2_waypoint3d_tool
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_waypoint3d_tool::Waypoint3DTool, rviz_common::Tool)

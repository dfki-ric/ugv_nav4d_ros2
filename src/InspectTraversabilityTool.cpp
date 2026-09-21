#include "InspectTraversabilityTool.hpp"

#include <cmath>
#include <QMetaObject>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_window.hpp>

namespace ugv_nav4d_ros2::ugv_nav4d_ros2_inspect_tool
{
InspectTraversabilityTool::InspectTraversabilityTool()
{
    shortcut_key_ = 'i';
}

void InspectTraversabilityTool::onInitialize()
{
    setName("Inspect Traversability");
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    client_ = node_->create_client<ugv_nav4d_ros2::srv::InspectTraversability>(
        "/ugv_nav4d_ros2/inspect_traversability");
    result_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/inspection_result", rclcpp::QoS(1).transient_local());
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    setStatus("Click a traversability patch to explain its classification.");
}

void InspectTraversabilityTool::activate()
{
    setStatus("Click a traversability patch to explain its classification.");
}

void InspectTraversabilityTool::deactivate()
{
}

int InspectTraversabilityTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
    if (!event.leftDown()) return 0;
    Ogre::Vector3 picked;
    if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, picked))
    {
        const auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
            event.panel->getRenderWindow(), event.x, event.y);
        if (!projection.first) return 0;
        picked = projection.second;
    }
    if (!client_->service_is_ready())
    {
        setStatus("Inspection service unavailable.");
        return 0;
    }
    auto request = std::make_shared<ugv_nav4d_ros2::srv::InspectTraversability::Request>();
    request->point.x = picked.x;
    request->point.y = picked.y;
    request->point.z = picked.z;
    client_->async_send_request(request,
        [this](rclcpp::Client<ugv_nav4d_ros2::srv::InspectTraversability>::SharedFuture future)
        {
            const auto response = future.get();
            QString text;
            if (!response->success)
            {
                text = QString::fromStdString(response->message);
            }
            else
            {
                text = QString("%1 | slope %2 deg | direction %3 deg | cost %4 | %5 allowed heading band(s)")
                    .arg(QString::fromStdString(response->type_name))
                    .arg(response->slope * 180.0 / M_PI, 0, 'f', 1)
                    .arg(response->slope_direction * 180.0 / M_PI, 0, 'f', 1)
                    .arg(response->cost)
                    .arg(response->allowed_orientation_starts.size());
            }
            std_msgs::msg::String result_message;
            result_message.data = text.toStdString();
            result_publisher_->publish(result_message);
            QMetaObject::invokeMethod(this, [this, text]() { setStatus(text); }, Qt::QueuedConnection);
        });
    return Render;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    ugv_nav4d_ros2::ugv_nav4d_ros2_inspect_tool::InspectTraversabilityTool,
    rviz_common::Tool)

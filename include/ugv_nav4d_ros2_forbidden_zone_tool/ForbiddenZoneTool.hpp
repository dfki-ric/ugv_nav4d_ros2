#ifndef UGV_NAV4D_ROS2_FORBIDDEN_ZONE_TOOL_HPP_
#define UGV_NAV4D_ROS2_FORBIDDEN_ZONE_TOOL_HPP_

#include <memory>
#include <vector>

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include "ugv_nav4d_ros2/msg/forbidden_zone.hpp"

namespace rviz_rendering
{
class BillboardLine;
class Shape;
}

namespace rviz_common
{
namespace properties
{
class StringProperty;
}
}

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_forbidden_zone_tool
{

/**
 * Draws a polygonal keep-out zone on the map: left-click on patches to add
 * vertices (minimum 3), right-click to close the polygon and send it to the
 * planner, which marks the covered trav nodes as obstacles. Deactivating the
 * tool discards an unfinished polygon. The tool stays active after closing a
 * polygon so several zones can be added in a row.
 */
class ForbiddenZoneTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    ForbiddenZoneTool();
    ~ForbiddenZoneTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private Q_SLOTS:
    void updateTopic();

private:
    void publishZone();
    void updatePreview();
    std::vector<Ogre::Vector3> orderedVertices() const;
    void resetPolygon();
    void updateStatusHint();

    std::vector<Ogre::Vector3> vertices_;
    std::shared_ptr<rviz_rendering::BillboardLine> outline_;
    std::vector<std::shared_ptr<rviz_rendering::Shape>> vertex_dots_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    rclcpp::Publisher<ugv_nav4d_ros2::msg::ForbiddenZone>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty* topic_property_;
};

} // namespace ugv_nav4d_ros2_forbidden_zone_tool
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_FORBIDDEN_ZONE_TOOL_HPP_

#ifndef UGV_NAV4D_ROS2_INSPECT_ORIENTATIONS_TOOL_HPP_
#define UGV_NAV4D_ROS2_INSPECT_ORIENTATIONS_TOOL_HPP_

#include <memory>
#include <vector>

#include <OgreVector.h>

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>

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
namespace ugv_nav4d_ros2_inspect_orientations_tool
{

/**
 * Selects an area on the map for orientation inspection: left-click on patches
 * to add polygon vertices (minimum 3), right-click to close and publish the
 * region. The planner node answers by rendering the allowed-orientation
 * wedges of every PARTIALLY_TRAVERSABLE cell inside the region. Right-click
 * with no vertices clears the markers.
 */
class InspectOrientationsTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    InspectOrientationsTool();
    ~InspectOrientationsTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private Q_SLOTS:
    void updateTopic();

private:
    void publishRegion(bool clear);
    void updatePreview();
    std::vector<Ogre::Vector3> orderedVertices() const;
    void resetPolygon();
    void updateStatusHint();

    std::vector<Ogre::Vector3> vertices_;
    std::shared_ptr<rviz_rendering::BillboardLine> outline_;
    std::vector<std::shared_ptr<rviz_rendering::Shape>> vertex_dots_;

    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
    rclcpp::Clock::SharedPtr clock_;

    rviz_common::properties::StringProperty* topic_property_;
};

} // namespace ugv_nav4d_ros2_inspect_orientations_tool
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_INSPECT_ORIENTATIONS_TOOL_HPP_

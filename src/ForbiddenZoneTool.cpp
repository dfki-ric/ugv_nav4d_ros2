#include "ForbiddenZoneTool.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <OgreSceneNode.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_window.hpp>

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_forbidden_zone_tool
{

ForbiddenZoneTool::ForbiddenZoneTool()
: rviz_common::Tool()
{
    shortcut_key_ = 'f';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/ugv_nav4d_ros2/add_forbidden_zone",
        "Topic on which the operational zone polygon is published.",
        getPropertyContainer(), SLOT(updateTopic()), this);
    zone_type_property_ = new rviz_common::properties::EnumProperty(
        "Zone type", "Keep-out", "How this polygon affects planning/execution.",
        getPropertyContainer());
    zone_type_property_->addOption("Keep-out", ugv_nav4d_ros2::msg::ForbiddenZone::KEEP_OUT);
    zone_type_property_->addOption("Caution / high cost", ugv_nav4d_ros2::msg::ForbiddenZone::CAUTION);
    zone_type_property_->addOption("Speed limit", ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT);
    zone_type_property_->addOption("Preferred corridor", ugv_nav4d_ros2::msg::ForbiddenZone::PREFERRED);
    zone_type_property_->addOption("Direction restricted", ugv_nav4d_ros2::msg::ForbiddenZone::DIRECTION_RESTRICTED);
    zone_type_property_->addOption("Annotation", ugv_nav4d_ros2::msg::ForbiddenZone::ANNOTATION);
    label_property_ = new rviz_common::properties::StringProperty(
        "Label", "", "Operator note displayed above the zone.", getPropertyContainer());
    cost_multiplier_property_ = new rviz_common::properties::FloatProperty(
        "Cost multiplier", 2.0, "Caution penalty or preferred-corridor divisor.", getPropertyContainer());
    cost_multiplier_property_->setMin(1.0);
    speed_limit_property_ = new rviz_common::properties::FloatProperty(
        "Speed limit", 0.25, "Execution speed limit in m/s.", getPropertyContainer());
    speed_limit_property_->setMin(0.0);
    heading_property_ = new rviz_common::properties::FloatProperty(
        "Preferred heading", 0.0, "Direction-zone heading in radians.", getPropertyContainer());
    duration_property_ = new rviz_common::properties::FloatProperty(
        "Duration", 0.0, "Zone lifetime in seconds; zero means no expiry.", getPropertyContainer());
    duration_property_->setMin(0.0);
}

ForbiddenZoneTool::~ForbiddenZoneTool() = default;

void ForbiddenZoneTool::onInitialize()
{
    outline_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, nullptr);
    outline_->setColor(1.0f, 0.1f, 0.1f, 1.0f);
    outline_->setLineWidth(0.1f);

    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

    setName("Add Operational Zone");
    updateTopic();
}

void ForbiddenZoneTool::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<ugv_nav4d_ros2::msg::ForbiddenZone>(
        topic_property_->getStdString(), 1);
    clock_ = raw_node->get_clock();
}

void ForbiddenZoneTool::activate()
{
    resetPolygon();
    updateStatusHint();
}

void ForbiddenZoneTool::deactivate()
{
    // An unfinished polygon is discarded.
    resetPolygon();
}

void ForbiddenZoneTool::resetPolygon()
{
    vertices_.clear();
    vertex_dots_.clear();
    if (outline_)
    {
        outline_->clear();
    }
}

void ForbiddenZoneTool::updateStatusHint()
{
    if (vertices_.size() < 3)
    {
        setStatus(QString("Left-click to add vertices (%1/3 minimum). Right-click to close the polygon.")
                      .arg(vertices_.size()));
    }
    else
    {
        setStatus(QString("%1 vertices. Left-click to add more, right-click to close the polygon.")
                      .arg(vertices_.size()));
    }
}

// Vertices sorted by angle around the centroid: the clicks can be made in any
// order and still yield a simple (non-self-intersecting) polygon. Click order
// with 4+ corner-hopping clicks would otherwise produce crossing edges.
std::vector<Ogre::Vector3> ForbiddenZoneTool::orderedVertices() const
{
    std::vector<Ogre::Vector3> sorted(vertices_.begin(), vertices_.end());
    if (sorted.size() < 3)
    {
        return sorted;
    }
    Ogre::Vector3 centroid(0.0f, 0.0f, 0.0f);
    for (const auto& v : sorted)
    {
        centroid += v;
    }
    centroid /= static_cast<float>(sorted.size());
    std::sort(sorted.begin(), sorted.end(),
              [&centroid](const Ogre::Vector3& a, const Ogre::Vector3& b)
              {
                  return std::atan2(a.y - centroid.y, a.x - centroid.x) <
                         std::atan2(b.y - centroid.y, b.x - centroid.x);
              });
    return sorted;
}

void ForbiddenZoneTool::updatePreview()
{
    outline_->clear();
    if (vertices_.empty())
    {
        return;
    }
    const auto ordered = orderedVertices();
    outline_->setLineWidth(0.1f);
    outline_->setMaxPointsPerLine(static_cast<uint32_t>(ordered.size() + 1));
    for (const auto& v : ordered)
    {
        outline_->addPoint(v);
    }
    if (ordered.size() >= 3)
    {
        outline_->addPoint(ordered.front()); // visual closing edge
    }
}

int ForbiddenZoneTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
    int flags = 0;

    if (event.leftDown())
    {
        // Pick the true 3D point on the rendered geometry (MLS/trav patches).
        // Falls back to the z=0 ground plane when clicking into empty space.
        Ogre::Vector3 picked;
        if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, picked))
        {
            auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);
            if (!projection.first)
            {
                return flags;
            }
            picked = projection.second;
        }

        vertices_.push_back(picked);

        auto dot = std::make_shared<rviz_rendering::Shape>(
            rviz_rendering::Shape::Sphere, scene_manager_, nullptr);
        dot->setColor(1.0f, 0.1f, 0.1f, 1.0f);
        dot->setPosition(picked);
        dot->setScale(Ogre::Vector3(0.25f, 0.25f, 0.25f));
        vertex_dots_.push_back(dot);

        updatePreview();
        updateStatusHint();
        flags |= Render;
    }
    else if (event.rightDown())
    {
        if (vertices_.size() >= 3)
        {
            publishZone();
            resetPolygon();
            updateStatusHint();
        }
        else
        {
            setStatus(QString("Polygon needs at least 3 vertices (%1 so far); left-click to add more.")
                          .arg(vertices_.size()));
        }
        flags |= Render;
    }

    return flags;
}

void ForbiddenZoneTool::publishZone()
{
    ugv_nav4d_ros2::msg::ForbiddenZone zone;
    zone.header.frame_id = context_->getFixedFrame().toStdString();
    zone.header.stamp = clock_->now();
    zone.zone_type = static_cast<uint8_t>(zone_type_property_->getOptionInt());
    zone.label = label_property_->getStdString();
    zone.cost_multiplier = cost_multiplier_property_->getFloat();
    zone.speed_limit = speed_limit_property_->getFloat();
    zone.preferred_heading = heading_property_->getFloat();
    if (duration_property_->getFloat() > 0.0f)
    {
        const rclcpp::Time expiry = clock_->now() + rclcpp::Duration::from_seconds(
            duration_property_->getFloat());
        const int64_t expiry_nanoseconds = expiry.nanoseconds();
        zone.expires_at.sec = static_cast<int32_t>(expiry_nanoseconds / 1000000000LL);
        zone.expires_at.nanosec = static_cast<uint32_t>(expiry_nanoseconds % 1000000000LL);
    }
    const auto ordered = orderedVertices();
    zone.vertices.reserve(ordered.size());
    for (const auto& v : ordered)
    {
        geometry_msgs::msg::Point p;
        p.x = v.x;
        p.y = v.y;
        p.z = v.z;
        zone.vertices.push_back(p);
    }

    RVIZ_COMMON_LOG_INFO_STREAM(
        "Publishing operational zone on " << topic_property_->getStdString()
        << ": " << zone.vertices.size() << " vertices, frame " << zone.header.frame_id);

    publisher_->publish(zone);
}

} // namespace ugv_nav4d_ros2_forbidden_zone_tool
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_forbidden_zone_tool::ForbiddenZoneTool, rviz_common::Tool)

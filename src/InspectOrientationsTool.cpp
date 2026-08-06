#include "InspectOrientationsTool.hpp"

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
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/render_window.hpp>

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_inspect_orientations_tool
{

InspectOrientationsTool::InspectOrientationsTool()
: rviz_common::Tool()
{
    shortcut_key_ = 'o';

    topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/ugv_nav4d_ros2/inspect_orientations_region",
        "Topic on which the inspection region polygon is published.",
        getPropertyContainer(), SLOT(updateTopic()), this);
}

InspectOrientationsTool::~InspectOrientationsTool() = default;

void InspectOrientationsTool::onInitialize()
{
    outline_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, nullptr);
    outline_->setColor(0.2f, 0.9f, 1.0f, 1.0f);
    outline_->setLineWidth(0.1f);

    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

    setName("Inspect Orientations");
    updateTopic();
}

void InspectOrientationsTool::updateTopic()
{
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<geometry_msgs::msg::PolygonStamped>(
        topic_property_->getStdString(), 1);
    clock_ = raw_node->get_clock();
}

void InspectOrientationsTool::activate()
{
    resetPolygon();
    updateStatusHint();
}

void InspectOrientationsTool::deactivate()
{
    // An unfinished polygon is discarded.
    resetPolygon();
}

void InspectOrientationsTool::resetPolygon()
{
    vertices_.clear();
    vertex_dots_.clear();
    if (outline_)
    {
        outline_->clear();
    }
}

void InspectOrientationsTool::updateStatusHint()
{
    if (vertices_.empty())
    {
        setStatus("Left-click to add region vertices; right-click with none to CLEAR the orientation markers.");
    }
    else if (vertices_.size() < 3)
    {
        setStatus(QString("Left-click to add vertices (%1/3 minimum). Right-click to inspect the region.")
                      .arg(vertices_.size()));
    }
    else
    {
        setStatus(QString("%1 vertices. Left-click to add more, right-click to inspect the region.")
                      .arg(vertices_.size()));
    }
}

// Vertices sorted by angle around the centroid: the clicks can be made in any
// order and still yield a simple (non-self-intersecting) polygon.
std::vector<Ogre::Vector3> InspectOrientationsTool::orderedVertices() const
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

void InspectOrientationsTool::updatePreview()
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

int InspectOrientationsTool::processMouseEvent(rviz_common::ViewportMouseEvent& event)
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
        dot->setColor(0.2f, 0.9f, 1.0f, 1.0f);
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
            publishRegion(false);
            resetPolygon();
            updateStatusHint();
        }
        else if (vertices_.empty())
        {
            // Explicit clear: an empty region removes the orientation markers.
            publishRegion(true);
            setStatus("Orientation markers cleared.");
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

void InspectOrientationsTool::publishRegion(bool clear)
{
    geometry_msgs::msg::PolygonStamped region;
    region.header.frame_id = context_->getFixedFrame().toStdString();
    region.header.stamp = clock_->now();
    if (!clear)
    {
        const auto ordered = orderedVertices();
        region.polygon.points.reserve(ordered.size());
        for (const auto& v : ordered)
        {
            geometry_msgs::msg::Point32 p;
            p.x = v.x;
            p.y = v.y;
            p.z = v.z;
            region.polygon.points.push_back(p);
        }
    }

    RVIZ_COMMON_LOG_INFO_STREAM(
        "Publishing orientation-inspection region on " << topic_property_->getStdString()
        << ": " << region.polygon.points.size() << " vertices, frame " << region.header.frame_id);

    publisher_->publish(region);
}

} // namespace ugv_nav4d_ros2_inspect_orientations_tool
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_inspect_orientations_tool::InspectOrientationsTool, rviz_common::Tool)

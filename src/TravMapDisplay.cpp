#include "TravMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <OgreSceneManager.h>

namespace ugv_nav4d_ros2 {

namespace ugv_nav4d_ros2_trav_map_plugin {

//Same palette as traversability_generator3d's TravMap3dVisualization
//(color_obstacles_by_cause) so both UIs read identically. Indexed by the
//ObstacleCause numeric value carried in TravPatch::obstacle_cause.
static Ogre::ColourValue obstacleCauseColor(uint8_t cause, const Ogre::ColourValue& fallback)
{
  switch (cause) {
    case 1: return Ogre::ColourValue(0.35f, 0.35f, 0.35f, 1.0f); // UNMEASURED: dark grey
    case 2: return Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f);    // STEEP_SLOPE: red
    case 3: return Ogre::ColourValue(0.63f, 0.13f, 0.94f, 1.0f); // STEP_HEIGHT: purple
    case 4: return Ogre::ColourValue(1.0f, 0.4f, 0.6f, 1.0f);    // INCLINE_LIMIT: pink
    case 5: return Ogre::ColourValue(0.55f, 0.0f, 0.0f, 1.0f);   // NO_SAFE_YAW: dark red
    case 6: return Ogre::ColourValue(0.55f, 0.35f, 0.15f, 1.0f); // MAP_BOUNDARY: brown
    default: return fallback;                                    // NONE / unknown
  }
}

TravMapDisplay::TravMapDisplay()
{
  color_by_cause_property_ = new rviz_common::properties::BoolProperty(
      "Color obstacles by cause", false,
      "Recolor obstacle patches by WHY they are obstacles: dark grey = unmeasured, "
      "red = steep slope, purple = step height, pink = incline limit, "
      "dark red = no safe yaw, brown = map boundary. Same palette as the travgen GUI.",
      this, SLOT(updateColorMode()));

  // In-rviz legend: read-only swatches in the property tree, so the operator
  // can check what a color means without leaving rviz. Values must match the
  // publisher (ugv_nav4d_ros2.cpp) and obstacleCauseColor() above.
  auto add_swatch = [](rviz_common::properties::Property* parent,
                       const char* name, QColor c, const char* desc) {
    auto* p = new rviz_common::properties::ColorProperty(name, c, desc, parent);
    p->setReadOnly(true);
  };
  auto* legend_types = new rviz_common::properties::Property(
      "Legend: cell types", QVariant(),
      "Reference only. Base colors of the published map.", this);
  legend_types->setReadOnly(true);
  add_swatch(legend_types, "Traversable", QColor(0, 255, 0), "Drivable in any heading");
  add_swatch(legend_types, "Partially traversable", QColor(153, 204, 0),
             "Drivable only in certain headings");
  add_swatch(legend_types, "Obstacle", QColor(255, 0, 0), "Not drivable");
  add_swatch(legend_types, "Inflated obstacle", QColor(255, 128, 0),
             "Blocked by the safety margin, not the cell itself");
  add_swatch(legend_types, "Frontier", QColor(0, 0, 255), "Edge of the explored map");
  add_swatch(legend_types, "Inflated frontier", QColor(128, 204, 255),
             "Safety margin around a frontier");
  add_swatch(legend_types, "Unknown", QColor(0, 140, 140), "Not yet classified");
  add_swatch(legend_types, "Unset", QColor(255, 255, 0), "Created but never evaluated");
  add_swatch(legend_types, "Hole / other", QColor(64, 82, 115), "No usable surface");

  auto* legend_causes = new rviz_common::properties::Property(
      "Legend: obstacle causes", QVariant(),
      "Reference only. Used when 'Color obstacles by cause' is enabled.", this);
  legend_causes->setReadOnly(true);
  add_swatch(legend_causes, "Unmeasured", QColor(89, 89, 89),
             "Ground-plane fit failed: no or too-sparse data");
  add_swatch(legend_causes, "Steep slope", QColor(255, 0, 0), "Slope exceeds maxSlope");
  add_swatch(legend_causes, "Step height", QColor(161, 33, 240),
             "Step collides with the robot body volume");
  add_swatch(legend_causes, "Incline limit", QColor(255, 102, 153),
             "No allowed heading under incline limiting");
  add_swatch(legend_causes, "No safe yaw", QColor(140, 0, 0),
             "Inflation found no collision-free heading");
  add_swatch(legend_causes, "Map boundary", QColor(140, 89, 38),
             "Robot body would leave the mapped grid");
}

TravMapDisplay::~TravMapDisplay()
{
  // Destructor implementation
}

void TravMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void TravMapDisplay::reset()
{
  MFDClass::reset();
  for (auto& object : manual_objects_) {
      scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();
  last_msg_.reset();
}

void TravMapDisplay::onDisable(){
  MFDClass::onDisable();
  for (auto& object : manual_objects_) {
      scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();
}

void TravMapDisplay::updateColorMode()
{
  //Re-render the cached map with the new color mode; the map topic is latched
  //and only republished on map changes, so waiting for the next message would
  //make the checkbox appear dead.
  if (last_msg_) {
    renderMap(*last_msg_);
  }
}

void TravMapDisplay::processMessage(ugv_nav4d_ros2::msg::TravMap::ConstSharedPtr msg)
{
  last_msg_ = msg;
  renderMap(*msg);
}

void TravMapDisplay::renderMap(const ugv_nav4d_ros2::msg::TravMap& msg)
{
  // Clear any previous visuals
  for (auto& object : manual_objects_) {
      scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();

  const bool color_by_cause = color_by_cause_property_ && color_by_cause_property_->getBool();

  // One ManualObject for the WHOLE map: a per-patch object means one scene
  // object and one draw call per cell, which freezes rviz on large maps
  // (and made the color-by-cause re-render appear to hang).
  auto* mesh = scene_manager_->createManualObject();
  mesh->setDynamic(false);
  manual_objects_.push_back(mesh);
  mesh->estimateVertexCount(msg.patches.size() * 4);
  mesh->estimateIndexCount(msg.patches.size() * 6);
  mesh->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  const float size = msg.resolution / 2;
  const Ogre::Vector3 default_normal(0, 0, 1);
  uint32_t base = 0;

  for (const auto& patch : msg.patches) {
    Ogre::Vector3 position(patch.position.x, patch.position.y, patch.position.z);

    Ogre::Vector3 normal(patch.a, patch.b, patch.c);
    normal.normalise();
    Ogre::Quaternion orientation = default_normal.getRotationTo(normal);

    Ogre::ColourValue color(patch.color.r, patch.color.g, patch.color.b, patch.color.a);
    if (color_by_cause && patch.obstacle_cause != 0) {
      color = obstacleCauseColor(patch.obstacle_cause, color);
    }

    const Ogre::Vector3 corners[4] = {
      position + orientation * Ogre::Vector3(-size, -size, 0),
      position + orientation * Ogre::Vector3(size, -size, 0),
      position + orientation * Ogre::Vector3(size, size, 0),
      position + orientation * Ogre::Vector3(-size, size, 0)
    };

    for (const auto& corner : corners) {
      mesh->position(corner);
      mesh->colour(color);
    }

    mesh->index(base + 0);
    mesh->index(base + 1);
    mesh->index(base + 2);
    mesh->index(base + 2);
    mesh->index(base + 3);
    mesh->index(base + 0);
    base += 4;
  }

  mesh->end();
  scene_node_->attachObject(mesh);
}

} // namespace ugv_nav4d_ros2_trav_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_trav_map_plugin::TravMapDisplay, rviz_common::Display)

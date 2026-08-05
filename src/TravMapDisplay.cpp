#include "TravMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
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

  for (const auto& patch : msg.patches) {
    // Convert the plane parameters to a visual representation
    Ogre::Vector3 position(patch.position.x, patch.position.y, patch.position.z);

    // Compute the normal vector of the plane (a, b, c) and a point on the plane
    Ogre::Vector3 normal(patch.a, patch.b, patch.c);
    normal.normalise();

    // Compute the orientation from the normal
    Ogre::Vector3 default_normal(0, 0, 1);
    Ogre::Quaternion orientation = default_normal.getRotationTo(normal);

    // Set the color of the plane
    Ogre::ColourValue color(patch.color.r, patch.color.g, patch.color.b, patch.color.a);
    if (color_by_cause && patch.obstacle_cause != 0) {
      color = obstacleCauseColor(patch.obstacle_cause, color);
    }

    // Create a plane visual for each patch
    auto plane = scene_manager_->createManualObject();
    manual_objects_.push_back(plane);
    plane->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Define the vertices of the plane based on the patch parameters
    // Example: A simple square plane, adjust according to your plane parameters
    float size = msg.resolution/2;

    Ogre::Vector3 corners[4] = {
      position + orientation * Ogre::Vector3(-size, -size, 0),
      position + orientation * Ogre::Vector3(size, -size, 0),
      position + orientation * Ogre::Vector3(size, size, 0),
      position + orientation * Ogre::Vector3(-size, size, 0)
    };

    plane->position(corners[0]);
    plane->colour(color);
    plane->position(corners[1]);
    plane->colour(color);
    plane->position(corners[2]);
    plane->colour(color);
    plane->position(corners[3]);
    plane->colour(color);

    plane->index(0);
    plane->index(1);
    plane->index(2);
    plane->index(2);
    plane->index(3);
    plane->index(0);

    plane->end();
    scene_node_->attachObject(plane);
  }
}

} // namespace ugv_nav4d_ros2_trav_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_trav_map_plugin::TravMapDisplay, rviz_common::Display)

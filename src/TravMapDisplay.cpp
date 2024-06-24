#include "TravMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

namespace ugv_nav4d_ros2 {

namespace ugv_nav4d_ros2_trav_map_plugin {

TravMapDisplay::TravMapDisplay()
{
  // Constructor implementation
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
}

void TravMapDisplay::processMessage(ugv_nav4d_ros2::msg::TravMap::ConstSharedPtr msg)
{
  // Clear any previous visuals
  scene_manager_->destroyAllManualObjects();

  for (const auto& patch : msg->patches) {
    // Convert the plane parameters to a visual representation
    Ogre::Vector3 position(patch.position.x, patch.position.y, patch.position.z);

    // Compute the normal vector of the plane (a, b, c) and a point on the plane
    Ogre::Vector3 normal(patch.a, patch.b, patch.c);
    normal.normalise();

    // Compute the orientation from the normal
    Ogre::Vector3 default_normal(0, 0, 1);
    Ogre::Quaternion orientation = default_normal.getRotationTo(normal);

    // Create a plane visual for each patch
    auto plane = scene_manager_->createManualObject();
    plane->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Define the vertices of the plane based on the patch parameters
    // Example: A simple square plane, adjust according to your plane parameters
    float size = 0.1; // Example size, set according to your grid cell size
    Ogre::Vector3 corners[4] = {
      position + orientation * Ogre::Vector3(-size, -size, 0),
      position + orientation * Ogre::Vector3(size, -size, 0),
      position + orientation * Ogre::Vector3(size, size, 0),
      position + orientation * Ogre::Vector3(-size, size, 0)
    };

    plane->position(corners[0]);
    plane->position(corners[1]);
    plane->position(corners[2]);
    plane->position(corners[3]);

    plane->index(0);
    plane->index(1);
    plane->index(2);
    plane->index(2);
    plane->index(3);
    plane->index(0);

    // Set the color of the plane
    Ogre::ColourValue color(patch.color.r, patch.color.g, patch.color.b, patch.color.a);
    plane->colour(color);

    plane->end();
    scene_node_->attachObject(plane);
  }
}

} // namespace ugv_nav4d_ros2_trav_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_trav_map_plugin::TravMapDisplay, rviz_common::Display)

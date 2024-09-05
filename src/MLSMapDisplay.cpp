#include "MLSMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

namespace ugv_nav4d_ros2 {

namespace ugv_nav4d_ros2_mls_map_plugin {

MLSMapDisplay::MLSMapDisplay()
{
  // Constructor implementation
}

MLSMapDisplay::~MLSMapDisplay()
{
  // Destructor implementation
}

void MLSMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void MLSMapDisplay::reset()
{
  MFDClass::reset();
}

void MLSMapDisplay::processMessage(ugv_nav4d_ros2::msg::MLSMap::ConstSharedPtr msg)
{
  // Clear any previous visuals
  scene_manager_->destroyAllManualObjects();

  std::cout << "Patches are " << msg->patches.size() << std::endl;

  for (const auto& patch : msg->patches) {
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

    // Create a plane visual for each patch
    auto plane = scene_manager_->createManualObject();
    plane->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    // Define the vertices of the plane based on the patch parameters
    // Example: A simple square plane, adjust according to your plane parameters
    float size = msg->resolution/2;

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
    
    // Back face (reverse order of vertices)
    plane->index(0);
    plane->index(3);
    plane->index(2);
    plane->index(2);
    plane->index(1);
    plane->index(0);

    plane->end();
    scene_node_->attachObject(plane);
  }
}

} // namespace ugv_nav4d_ros2_mls_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_mls_map_plugin::MLSMapDisplay, rviz_common::Display)

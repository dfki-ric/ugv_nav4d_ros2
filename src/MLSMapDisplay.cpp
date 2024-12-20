#include "MLSMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <OgreSceneManager.h>

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
  for (auto& object : manual_objects_) {
      scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();

  std::cout << "Patches are " << msg->patches.size() << std::endl;

  // Variables to keep track of height range
  float min_height = std::numeric_limits<float>::max();
  float max_height = std::numeric_limits<float>::min();

  // First pass to find the min and max height
  for (const auto& patch : msg->patches) {
      if (patch.position.z < min_height) {
          min_height = patch.position.z;
      }
      if (patch.position.z > max_height) {
          max_height = patch.position.z;
      }
  }


  for (const auto& patch : msg->patches) {
    // Convert the plane parameters to a visual representation
    Ogre::Vector3 position(patch.position.x, patch.position.y, patch.position.z);

    // Compute the normal vector of the plane (a, b, c) and a point on the plane
    Ogre::Vector3 normal(patch.a, patch.b, patch.c);
    normal.normalise();

    // Compute the orientation from the normal
    Ogre::Vector3 default_normal(0, 0, 1);
    Ogre::Quaternion orientation = default_normal.getRotationTo(normal);


    // Normalize the height of the patch between 0 and 1
    float height_normalized = (patch.position.z - min_height) / (max_height - min_height);

    // Map the normalized height to a color gradient (e.g., blue at low heights, red at high heights)
    Ogre::ColourValue color;
    color.r = height_normalized;   // Red increases with height
    color.g = 1.0f;                // Green remains constant (or can be adjusted if needed)
    color.b = 1.0f - height_normalized; // Blue decreases with height
    color.a = 1.0f;                // Fully opaque

    // Create a plane visual for each patch
    auto plane = scene_manager_->createManualObject();
    manual_objects_.push_back(plane);
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

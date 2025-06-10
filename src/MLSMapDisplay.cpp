#include "MLSMapDisplay.hpp"
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <OgreSceneManager.h>

#include <Eigen/Dense>
#include <maps/tools/SurfaceIntersection.hpp>

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
    Ogre::Vector3 position(patch.position.x, patch.position.y, patch.position.z);
    Ogre::Vector3 normal(patch.a, patch.b, patch.c);
    normal.normalise();

    Ogre::Vector3 default_normal(0, 0, 1);
    Ogre::Quaternion orientation = default_normal.getRotationTo(normal);

    float height_normalized = (patch.position.z - min_height) / (max_height - min_height);

    // Map the normalized height to a color gradient (e.g., blue at low heights, red at high heights)
    Ogre::ColourValue color;
    color.r = height_normalized;      // Red increases with height
    color.g = 1.0f;                   // Green remains constant
    color.b = 1.0f - height_normalized; // Blue decreases with height
    color.a = 1.0f;                   // Fully opaque

    // Create a manual object for visualization
    auto object = scene_manager_->createManualObject();
    manual_objects_.push_back(object);
    object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    float size = msg->resolution / 2;

    const Eigen::AlignedBox<float, 3> box(Eigen::Vector3f(-size, -size, patch.minz), Eigen::Vector3f(size, size, patch.maxz));
    std::vector< Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > intersections;
    Eigen::Vector3f normal_eigen{normal.x, normal.y, normal.z};
    float d = patch.d;
    Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal_eigen, -d);
    maps::tools::SurfaceIntersection::computeIntersections(plane, box, intersections);

    if (!intersections.empty()) {
      // --- Draw Intersections ---
      for (const auto& point : intersections) {
          object->position(patch.position.x + point.x(), 
                           patch.position.y + point.y(), 
                           point.z());

          object->colour(color);
      }

      for (size_t i = 1; i < intersections.size()-1; ++i) {
          object->index(0);
          object->index(i);
          object->index(i+1);

          object->index(i+1);
          object->index(i);
          object->index(0);

      }
    }
    object->end();
    scene_node_->attachObject(object);
  }
}

} // namespace ugv_nav4d_ros2_mls_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_mls_map_plugin::MLSMapDisplay, rviz_common::Display)

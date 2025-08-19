#include "MLSMapDisplay.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>

#include <Eigen/Dense>
#include <maps/tools/SurfaceIntersection.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugv_nav4d_ros2 {
namespace ugv_nav4d_ros2_mls_map_plugin {

MLSMapDisplay::MLSMapDisplay() {}
MLSMapDisplay::~MLSMapDisplay() {}

void MLSMapDisplay::onInitialize()
{
  MFDClass::onInitialize();

  // ---- UI Properties ----
  color_mode_property_ = new rviz_common::properties::EnumProperty(
      "Color Mode", "Height (centroid Z)",
      "Select which metric determines the patch color.",
      this, SLOT(updateColoring()));

  color_mode_property_->addOption("Height (centroid Z)", static_cast<int>(ColorMode::Height));
  color_mode_property_->addOption("Thickness (maxz - minz)", static_cast<int>(ColorMode::Thickness));
  color_mode_property_->addOption("Normal-Z (n·z)", static_cast<int>(ColorMode::NormalZ));
  color_mode_property_->addOption("Slope (deg)", static_cast<int>(ColorMode::Slope));

  colormap_property_ = new rviz_common::properties::EnumProperty(
      "Colormap", "Jet",
      "Colormap used to convert the normalized value to color.",
      this, SLOT(updateColoring()));

  colormap_property_->addOption("Turbo", static_cast<int>(Colormap::Turbo));
  colormap_property_->addOption("Jet", static_cast<int>(Colormap::Jet));
  colormap_property_->addOption("Viridis", static_cast<int>(Colormap::Viridis));
  colormap_property_->addOption("Grayscale", static_cast<int>(Colormap::Gray));
  colormap_property_->addOption("CyanYellow (legacy)", static_cast<int>(Colormap::CyanYellow));

  auto_range_property_ = new rviz_common::properties::BoolProperty(
      "Auto Range", true,
      "If enabled, min/max are computed from each incoming message.",
      this, SLOT(updateColoring()));

  min_value_property_ = new rviz_common::properties::FloatProperty(
      "Min Value", 0.0,
      "Lower bound of value range (used when Auto Range is disabled).",
      this, SLOT(updateColoring()));
  max_value_property_ = new rviz_common::properties::FloatProperty(
      "Max Value", 1.0,
      "Upper bound of value range (used when Auto Range is disabled).",
      this, SLOT(updateColoring()));

  // Wide bounds so users can dial in whatever they need
  min_value_property_->setMin(-1e9);
  max_value_property_->setMax(+1e9);
}

void MLSMapDisplay::reset()
{
  MFDClass::reset();
  for (auto* object : manual_objects_) {
    scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();
}

void MLSMapDisplay::onDisable()
{
  MFDClass::onDisable();
  for (auto* object : manual_objects_) {
    scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();
}

void MLSMapDisplay::updateColoring()
{
  color_mode_ = static_cast<ColorMode>(color_mode_property_->getOptionInt());
  colormap_   = static_cast<Colormap>(colormap_property_->getOptionInt());
  auto_range_ = auto_range_property_->getBool();
  user_min_   = min_value_property_->getFloat();
  user_max_   = max_value_property_->getFloat();

  // Optionally hide min/max when Auto Range is on (keeps UI tidy)
  const bool hide_fixed = auto_range_;
  min_value_property_->setHidden(hide_fixed);
  max_value_property_->setHidden(hide_fixed);

}

// Helpers

float MLSMapDisplay::safeNormalize(float v, float vmin, float vmax)
{
  const float eps = 1e-6f;
  const float range = std::max(vmax - vmin, eps);
  float t = (v - vmin) / range;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return t;
}

float MLSMapDisplay::metricForPatch(const ugv_nav4d_ros2::msg::MLSPatch& p) const
{
  switch (color_mode_) {
    case ColorMode::Height:
      return p.position.z;

    case ColorMode::Thickness:
      return p.maxz - p.minz;

    case ColorMode::NormalZ: {
      Eigen::Vector3f n(p.a, p.b, p.c);
      if (n.norm() < 1e-9f) return 0.5f; // mid if invalid normal
      n.normalize();
      // Map [-1,1] to [0,1] directly as the metric (already normalized)
      return 0.5f * (n.z() + 1.0f);
    }

    case ColorMode::Slope: {
      Eigen::Vector3f n(p.a, p.b, p.c);
      if (n.norm() < 1e-9f) return 90.0f; // treat the patch as vertical
      n.normalize();
      const float cos_t = std::clamp(n.dot(Eigen::Vector3f::UnitZ()), -1.0f, 1.0f);
      const float theta = std::acos(cos_t); // radians
      return theta * 180.0f / float(M_PI);  // degrees
    }
  }
  // Fallback
  return p.position.z;
}

Ogre::ColourValue MLSMapDisplay::mapToColor(float t) const
{
  t = std::clamp(t, 0.0f, 1.0f);

  switch (colormap_) {
    case Colormap::Gray: {
      return Ogre::ColourValue(t, t, t, 1.0f);
    }
    case Colormap::CyanYellow: {
      // legacy palette: cyan (0,1,1) → yellow (1,1,0)
      return Ogre::ColourValue(t, 1.0f, 1.0f - t, 1.0f);
    }
    case Colormap::Jet: {
      // compact Jet approximation
      float r = std::clamp(1.5f * t - 0.5f, 0.0f, 1.0f);
      float g = std::clamp(1.5f - std::fabs(2.0f * t - 1.0f), 0.0f, 1.0f);
      float b = std::clamp(1.5f * (1.0f - t) - 0.5f, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }
    case Colormap::Viridis: {
      // small polynomial fit (approx)
      float r = std::clamp(0.2777f + 0.1050f*t - 0.3309f*t*t + 0.9850f*t*t*t, 0.0f, 1.0f);
      float g = std::clamp(0.0054f + 1.4044f*t - 1.4537f*t*t + 0.7180f*t*t*t, 0.0f, 1.0f);
      float b = std::clamp(0.3340f + 1.3845f*t - 2.0906f*t*t + 1.0641f*t*t*t, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }
    case Colormap::Turbo:
    default: {
      // Google Turbo (polynomial approximation)
      const float r = std::clamp(0.135f + 4.615f*t - 42.660f*t*t + 132.131f*t*t*t - 152.942f*t*t*t*t + 59.286f*t*t*t*t*t, 0.0f, 1.0f);
      const float g = std::clamp(0.091f + 2.086f*t + 4.021f*t*t - 25.834f*t*t*t + 39.995f*t*t*t*t - 19.332f*t*t*t*t*t, 0.0f, 1.0f);
      const float b = std::clamp(0.106f + 2.532f*t - 1.179f*t*t - 15.950f*t*t*t + 33.504f*t*t*t*t - 17.228f*t*t*t*t*t, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }
  }
}

// Rendering

void MLSMapDisplay::processMessage(ugv_nav4d_ros2::msg::MLSMap::ConstSharedPtr msg)
{
  // Clear previous visuals
  for (auto* object : manual_objects_) {
    scene_manager_->destroyManualObject(object);
  }
  manual_objects_.clear();

  // Compute range
  float vmin = std::numeric_limits<float>::infinity();
  float vmax = -std::numeric_limits<float>::infinity();

  if (auto_range_) {
    for (const auto& patch : msg->patches) {
      const float m = metricForPatch(patch);
      vmin = std::min(vmin, m);
      vmax = std::max(vmax, m);
    }
    if (!std::isfinite(vmin) || !std::isfinite(vmax)) {
      vmin = 0.0f; vmax = 1.0f;
    }
    if (std::fabs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1e-3f; // avoid zero range
    }
  } else {
    vmin = std::min(user_min_, user_max_);
    vmax = std::max(user_min_, user_max_);
    if (std::fabs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1e-3f;
    }
  }

  // Build each patch
  for (const auto& patch : msg->patches) {
    // Normal (unit) for plane
    Ogre::Vector3 n_ogre(patch.a, patch.b, patch.c);
    if (n_ogre.squaredLength() > 1e-12f) n_ogre.normalise();

    // Metric + color
    const float metric = metricForPatch(patch);
    const float t      = safeNormalize(metric, vmin, vmax);
    const Ogre::ColourValue color = mapToColor(t);

    // Create manual object
    Ogre::ManualObject* object = scene_manager_->createManualObject();
    manual_objects_.push_back(object);

    object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    const float half = msg->resolution / 2.0f;

    const Eigen::AlignedBox<float, 3> box(
        Eigen::Vector3f(-half, -half, patch.minz),
        Eigen::Vector3f( half,  half, patch.maxz));

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> verts;
    Eigen::Vector3f n_eigen(n_ogre.x, n_ogre.y, n_ogre.z);

    // Eigen plane expects offset; patches use ax + by + cz + d = 0
    // Eigen::Hyperplane uses n·x + offset = 0 → offset = d, but constructor with (normal, scalar)
    // can be given as (n, -d) to match ax + by + cz - d = 0; here patches store 'd' consistent with ax + by + cz + d = 0
    // We want plane(n, -d) if d is the plane constant on LHS. Adjust if your sign convention differs.
    const float d = patch.d;
    Eigen::Hyperplane<float, 3> plane(n_eigen, -d);

    maps::tools::SurfaceIntersection::computeIntersections(plane, box, verts);

    if (!verts.empty()) {
      // Vertices with uniform per-patch color (fastest)
      for (const auto& v : verts) {
        object->position(patch.position.x + v.x(),
                         patch.position.y + v.y(),
                         v.z());
        object->colour(color);
      }

      // Fan two sided triangulation: (0, i, i+1) and (i+1, i, 0)
      for (size_t i = 1; i + 1 < verts.size(); ++i) {
        object->index(0); object->index(i); object->index(i+1);
        object->index(i+1); object->index(i); object->index(0);
      }
    }

    object->end();
    scene_node_->attachObject(object);
  }
}

} // namespace ugv_nav4d_ros2_mls_map_plugin
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  ugv_nav4d_ros2::ugv_nav4d_ros2_mls_map_plugin::MLSMapDisplay,
  rviz_common::Display)

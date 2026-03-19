#include "MLSMapDisplay.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <Eigen/Dense>
#include <maps/tools/SurfaceIntersection.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace ugv_nav4d_ros2 {
namespace ugv_nav4d_ros2_mls_map_plugin {

MLSMapDisplay::MLSMapDisplay()
: manual_object_(nullptr)
{
}

MLSMapDisplay::~MLSMapDisplay()
{
}

void MLSMapDisplay::onInitialize()
{
  MFDClass::onInitialize();

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

  min_value_property_->setMin(-1e9);
  max_value_property_->setMax(+1e9);

  color_mode_ = ColorMode::Height;
  colormap_ = Colormap::Jet;
  auto_range_ = true;
  user_min_ = 0.0f;
  user_max_ = 1.0f;

  if (!manual_object_) {
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node_->attachObject(manual_object_);
  }
}

void MLSMapDisplay::reset()
{
  MFDClass::reset();

  last_msg_.reset();
  cached_metrics_.clear();

  if (manual_object_) {
    manual_object_->clear();
  }
}

void MLSMapDisplay::onDisable()
{
  MFDClass::onDisable();

  if (manual_object_) {
    manual_object_->clear();
  }
}

void MLSMapDisplay::updateColoring()
{
  color_mode_ = static_cast<ColorMode>(color_mode_property_->getOptionInt());
  colormap_   = static_cast<Colormap>(colormap_property_->getOptionInt());
  auto_range_ = auto_range_property_->getBool();
  user_min_   = min_value_property_->getFloat();
  user_max_   = max_value_property_->getFloat();

  const bool hide_fixed = auto_range_;
  min_value_property_->setHidden(hide_fixed);
  max_value_property_->setHidden(hide_fixed);

  if (last_msg_) {
    rebuildGeometry(*last_msg_);
  }
}

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
      return static_cast<float>(p.position.z);

    case ColorMode::Thickness:
      return static_cast<float>(p.maxz - p.minz);

    case ColorMode::NormalZ: {
      const float nx = static_cast<float>(p.a);
      const float ny = static_cast<float>(p.b);
      const float nz = static_cast<float>(p.c);
      const float norm_sq = nx * nx + ny * ny + nz * nz;
      if (norm_sq < 1e-18f) {
        return 0.5f;
      }
      const float inv_norm = 1.0f / std::sqrt(norm_sq);
      const float unit_nz = nz * inv_norm;
      return 0.5f * (unit_nz + 1.0f);
    }

    case ColorMode::Slope: {
      const float nx = static_cast<float>(p.a);
      const float ny = static_cast<float>(p.b);
      const float nz = static_cast<float>(p.c);
      const float norm_sq = nx * nx + ny * ny + nz * nz;
      if (norm_sq < 1e-18f) {
        return 90.0f;
      }
      const float inv_norm = 1.0f / std::sqrt(norm_sq);
      const float cos_t = std::clamp(nz * inv_norm, -1.0f, 1.0f);
      return std::acos(cos_t) * 180.0f / static_cast<float>(M_PI);
    }
  }

  return static_cast<float>(p.position.z);
}

Ogre::ColourValue MLSMapDisplay::mapToColor(float t) const
{
  t = std::clamp(t, 0.0f, 1.0f);

  switch (colormap_) {
    case Colormap::Gray:
      return Ogre::ColourValue(t, t, t, 1.0f);

    case Colormap::CyanYellow:
      return Ogre::ColourValue(t, 1.0f, 1.0f - t, 1.0f);

    case Colormap::Jet: {
      const float r = std::clamp(1.5f * t - 0.5f, 0.0f, 1.0f);
      const float g = std::clamp(1.5f - std::fabs(2.0f * t - 1.0f), 0.0f, 1.0f);
      const float b = std::clamp(1.5f * (1.0f - t) - 0.5f, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }

    case Colormap::Viridis: {
      const float r = std::clamp(0.2777f + 0.1050f * t - 0.3309f * t * t + 0.9850f * t * t * t, 0.0f, 1.0f);
      const float g = std::clamp(0.0054f + 1.4044f * t - 1.4537f * t * t + 0.7180f * t * t * t, 0.0f, 1.0f);
      const float b = std::clamp(0.3340f + 1.3845f * t - 2.0906f * t * t + 1.0641f * t * t * t, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }

    case Colormap::Turbo:
    default: {
      const float t2 = t * t;
      const float t3 = t2 * t;
      const float t4 = t3 * t;
      const float t5 = t4 * t;

      const float r = std::clamp(0.135f + 4.615f * t - 42.660f * t2 + 132.131f * t3 - 152.942f * t4 + 59.286f * t5, 0.0f, 1.0f);
      const float g = std::clamp(0.091f + 2.086f * t + 4.021f * t2 - 25.834f * t3 + 39.995f * t4 - 19.332f * t5, 0.0f, 1.0f);
      const float b = std::clamp(0.106f + 2.532f * t - 1.179f * t2 - 15.950f * t3 + 33.504f * t4 - 17.228f * t5, 0.0f, 1.0f);
      return Ogre::ColourValue(r, g, b, 1.0f);
    }
  }
}

void MLSMapDisplay::processMessage(ugv_nav4d_ros2::msg::MLSMap::ConstSharedPtr msg)
{
  last_msg_ = msg;
  rebuildGeometry(*msg);
}

void MLSMapDisplay::rebuildGeometry(const ugv_nav4d_ros2::msg::MLSMap& msg)
{
  if (!manual_object_) {
    manual_object_ = scene_manager_->createManualObject();
    manual_object_->setDynamic(true);
    scene_node_->attachObject(manual_object_);
  }

  manual_object_->clear();

  const std::size_t num_patches = msg.patches.size();
  if (num_patches == 0) {
    return;
  }

  cached_metrics_.resize(num_patches);

  float vmin = std::numeric_limits<float>::infinity();
  float vmax = -std::numeric_limits<float>::infinity();

  for (std::size_t i = 0; i < num_patches; ++i) {
    const float m = metricForPatch(msg.patches[i]);
    cached_metrics_[i] = m;

    if (auto_range_) {
      vmin = std::min(vmin, m);
      vmax = std::max(vmax, m);
    }
  }

  if (auto_range_) {
    if (!std::isfinite(vmin) || !std::isfinite(vmax)) {
      vmin = 0.0f;
      vmax = 1.0f;
    }
    if (std::fabs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1e-3f;
    }
  } else {
    vmin = std::min(user_min_, user_max_);
    vmax = std::max(user_min_, user_max_);
    if (std::fabs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1e-3f;
    }
  }

  manual_object_->estimateVertexCount(static_cast<unsigned int>(num_patches * 8));
  manual_object_->estimateIndexCount(static_cast<unsigned int>(num_patches * 24));

  manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  const float half = static_cast<float>(msg.resolution) * 0.5f;
  std::size_t vertex_base = 0;

  for (std::size_t i = 0; i < num_patches; ++i) {
    const auto& patch = msg.patches[i];

    const float metric = cached_metrics_[i];
    const float t = safeNormalize(metric, vmin, vmax);
    const Ogre::ColourValue color = mapToColor(t);

    const float a = static_cast<float>(patch.a);
    const float b = static_cast<float>(patch.b);
    const float c = static_cast<float>(patch.c);
    const float d = static_cast<float>(patch.d);
    const float minz = static_cast<float>(patch.minz);
    const float maxz = static_cast<float>(patch.maxz);
    const float px = static_cast<float>(patch.position.x);
    const float py = static_cast<float>(patch.position.y);

    // Hybrid path:
    // - near-vertical patches use exact clipped polygon intersection
    // - other patches use very fast quad projection
    const bool near_vertical = std::fabs(c) < 0.3f;

    if (near_vertical) {
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> verts;
      verts.reserve(16);

      Eigen::Vector3f n(a, b, c);
      if (n.squaredNorm() < 1e-12f) {
        continue;
      }

      Eigen::Hyperplane<float, 3> plane(n, -d);
      const Eigen::AlignedBox<float, 3> box(
          Eigen::Vector3f(-half, -half, minz),
          Eigen::Vector3f( half,  half, maxz));

      maps::tools::SurfaceIntersection::computeIntersections(plane, box, verts);

      if (verts.size() < 3) {
        continue;
      }

      const std::size_t base = vertex_base;
      for (const auto& v : verts) {
        manual_object_->position(px + v.x(), py + v.y(), v.z());
        manual_object_->colour(color);
      }

      for (std::size_t k = 1; k + 1 < verts.size(); ++k) {
        manual_object_->index(static_cast<uint32_t>(base + 0));
        manual_object_->index(static_cast<uint32_t>(base + k));
        manual_object_->index(static_cast<uint32_t>(base + k + 1));

        manual_object_->index(static_cast<uint32_t>(base + k + 1));
        manual_object_->index(static_cast<uint32_t>(base + k));
        manual_object_->index(static_cast<uint32_t>(base + 0));
      }

      vertex_base += verts.size();
      continue;
    }

    // Fast path: project cell corners to plane
    const float lx0 = -half;
    const float ly0 = -half;
    const float lx1 =  half;
    const float ly1 = -half;
    const float lx2 =  half;
    const float ly2 =  half;
    const float lx3 = -half;
    const float ly3 =  half;

    auto planeZLocal = [&](float lx, float ly) -> float {
      return -(a * lx + b * ly + d) / c;
    };

    float z0 = planeZLocal(lx0, ly0);
    float z1 = planeZLocal(lx1, ly1);
    float z2 = planeZLocal(lx2, ly2);
    float z3 = planeZLocal(lx3, ly3);

    // Preserve planar consistency:
    // shift the whole patch if its center lies outside [minz, maxz]
    // instead of clamping each corner independently.
    const float z_center = -d / c;

    if (z_center < minz) {
      const float shift = minz - z_center;
      z0 += shift;
      z1 += shift;
      z2 += shift;
      z3 += shift;
    } else if (z_center > maxz) {
      const float shift = maxz - z_center;
      z0 += shift;
      z1 += shift;
      z2 += shift;
      z3 += shift;
    }

    // Safety clamp after the planar shift
    z0 = std::clamp(z0, minz, maxz);
    z1 = std::clamp(z1, minz, maxz);
    z2 = std::clamp(z2, minz, maxz);
    z3 = std::clamp(z3, minz, maxz);

    manual_object_->position(px + lx0, py + ly0, z0);
    manual_object_->colour(color);

    manual_object_->position(px + lx1, py + ly1, z1);
    manual_object_->colour(color);

    manual_object_->position(px + lx2, py + ly2, z2);
    manual_object_->colour(color);

    manual_object_->position(px + lx3, py + ly3, z3);
    manual_object_->colour(color);

    manual_object_->index(static_cast<uint32_t>(vertex_base + 0));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 1));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 2));

    manual_object_->index(static_cast<uint32_t>(vertex_base + 0));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 2));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 3));

    manual_object_->index(static_cast<uint32_t>(vertex_base + 2));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 1));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 0));

    manual_object_->index(static_cast<uint32_t>(vertex_base + 3));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 2));
    manual_object_->index(static_cast<uint32_t>(vertex_base + 0));

    vertex_base += 4;
  }

  manual_object_->end();
}

}  // namespace ugv_nav4d_ros2_mls_map_plugin
}  // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    ugv_nav4d_ros2::ugv_nav4d_ros2_mls_map_plugin::MLSMapDisplay,
    rviz_common::Display)
#ifndef UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP
#define UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <ugv_nav4d_ros2/msg/mls_map.hpp>

#include <OgreManualObject.h>
#include <vector>

namespace ugv_nav4d_ros2 {
namespace ugv_nav4d_ros2_mls_map_plugin {

class MLSMapDisplay : public rviz_common::MessageFilterDisplay<ugv_nav4d_ros2::msg::MLSMap>
{
  Q_OBJECT
public:
  MLSMapDisplay();
  ~MLSMapDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void onDisable() override;

  void processMessage(ugv_nav4d_ros2::msg::MLSMap::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateColoring();

private:
  // ---- Coloring configuration ----
  enum class ColorMode {Height = 0, Thickness = 1, NormalZ = 2, Slope = 3};
  enum class Colormap  {Turbo = 0, Jet = 1, Viridis = 2, Gray = 3, CyanYellow = 4};

  // Property widgets shown in RViz
  rviz_common::properties::EnumProperty*  color_mode_property_{nullptr};
  rviz_common::properties::EnumProperty*  colormap_property_{nullptr};
  rviz_common::properties::BoolProperty*  auto_range_property_{nullptr};
  rviz_common::properties::FloatProperty* min_value_property_{nullptr};
  rviz_common::properties::FloatProperty* max_value_property_{nullptr};

  // Cached values derived from properties
  ColorMode color_mode_{ColorMode::Height};
  Colormap  colormap_{Colormap::Jet};
  bool      auto_range_{true};
  float     user_min_{0.0f};
  float     user_max_{1.0f};

  // Rendering
  std::vector<Ogre::ManualObject*> manual_objects_;

  // Helpers
  float metricForPatch(const ugv_nav4d_ros2::msg::MLSPatch& p) const;
  static float safeNormalize(float v, float vmin, float vmax);
  Ogre::ColourValue mapToColor(float t) const;
};

} // namespace ugv_nav4d_ros2_mls_map_plugin
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP

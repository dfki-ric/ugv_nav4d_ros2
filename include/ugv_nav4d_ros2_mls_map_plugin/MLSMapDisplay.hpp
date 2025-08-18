#ifndef UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP
#define UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP

#include <rviz_common/display.hpp>
#include <ugv_nav4d_ros2/msg/mls_map.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <memory>
#include <vector>

#include <OgreManualObject.h>

namespace ugv_nav4d_ros2 {

namespace ugv_nav4d_ros2_mls_map_plugin{

class MLSMapDisplay : public rviz_common::MessageFilterDisplay<ugv_nav4d_ros2::msg::MLSMap>
{
  Q_OBJECT
public:
  MLSMapDisplay();
  virtual ~MLSMapDisplay();

protected:
  virtual void onInitialize() override;
  virtual void reset() override;
  virtual void onDisable() override;

private:
  void processMessage(ugv_nav4d_ros2::msg::MLSMap::ConstSharedPtr msg) override;
  std::vector<Ogre::ManualObject*> manual_objects_;
};

} // ugv_nav4d_ros2_mls_map_plugin

} // ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_MLS_MAP_DISPLAY_HPP

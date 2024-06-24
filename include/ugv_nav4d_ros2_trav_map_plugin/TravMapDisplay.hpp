#ifndef UGV_NAV4D_ROS2_TRAV_MAP_DISPLAY_HPP
#define UGV_NAV4D_ROS2_TRAV_MAP_DISPLAY_HPP

#include <rviz_common/display.hpp>
#include <ugv_nav4d_ros2/msg/trav_map.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <memory>

namespace ugv_nav4d_ros2 {

namespace ugv_nav4d_ros2_trav_map_plugin{

class TravMapDisplay : public rviz_common::MessageFilterDisplay<ugv_nav4d_ros2::msg::TravMap>
{
  Q_OBJECT
public:
  TravMapDisplay();
  virtual ~TravMapDisplay();

protected:
  virtual void onInitialize() override;
  virtual void reset() override;

private:
  void processMessage(ugv_nav4d_ros2::msg::TravMap::ConstSharedPtr msg) override;
};

} // ugv_nav4d_ros2_trav_map_plugin

} // ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_TRAV_MAP_DISPLAY_HPP

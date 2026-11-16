cmake_minimum_required(VERSION 3.8)
project(ugv_nav4d_ros2)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

set(ugv_nav4d_ros2_MAJOR_VERSION 1)
set(ugv_nav4d_ros2_MINOR_VERSION 0)
set(ugv_nav4d_ros2_PATCH_VERSION 0)
set(ugv_nav4d_ros2_VERSION
  ${ugv_nav4d_ros2_MAJOR_VERSION}.${ugv_nav4d_ros2_MINOR_VERSION}.${ugv_nav4d_ros2_PATCH_VERSION})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenMP)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ugv_nav4d REQUIRED IMPORTED_TARGET ugv_nav4d)

# Strip conflicting C++ standard flags exported by the Rock dependencies
foreach(pkg_target IN ITEMS PkgConfig::ugv_nav4d)
  if (TARGET ${pkg_target})
    get_target_property(pkg_opts ${pkg_target} INTERFACE_COMPILE_OPTIONS)
    if (pkg_opts)
      list(REMOVE_ITEM pkg_opts "-std=c++11" "-std=c++14" "-std=gnu++11" "-std=gnu++14")
      set_target_properties(${pkg_target} PROPERTIES INTERFACE_COMPILE_OPTIONS "${pkg_opts}")
    endif()
  endif()
endforeach()

rosidl_generate_interfaces("ugv_nav4d_ros2_msgs"
  "msg/TravPatch.msg"
  "msg/TravMap.msg"
  "msg/MLSPatch.msg"
  "msg/MLSMap.msg"
  "msg/LabeledPathArray.msg"
  "msg/ForbiddenZone.msg"
  "msg/MissionStatus.msg"
  "msg/RouteRisk.msg"
  "msg/SystemHealth.msg"
  "msg/OperationalZoneArray.msg"
  "srv/EditWaypoint.srv"
  "srv/DeleteForbiddenZone.srv"
  "srv/SetFillPlane.srv"
  "srv/DeleteMlsPatches.srv"
  "srv/InspectTraversability.srv"
  "srv/MissionFile.srv"
  "action/SaveMLSMap.action"
  DEPENDENCIES geometry_msgs std_msgs nav_msgs builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  "ugv_nav4d_ros2_msgs" "rosidl_typesupport_cpp")

# Planner library
add_library(${PROJECT_NAME} SHARED src/ugv_nav4d_ros2.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # pcl_conversions without exported targets (before kilted)
  ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${pcl_conversions_TARGETS}
  ${pcl_conversions_LIBRARIES}
  Eigen3::Eigen
  Boost::serialization
  PkgConfig::ugv_nav4d
)
# omp_set_num_threads(numThreads) in ugv_nav4d_ros2.cpp caps the traversability
# generator's wave-parallel expansion with the SAME numThreads parameter the
# planner uses. Without OpenMP the call compiles out (and travgen runs serial).
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(ugv_nav4d_ros2_node src/ugv_nav4d_ros2_node.cpp)
target_link_libraries(ugv_nav4d_ros2_node PRIVATE ${PROJECT_NAME})

# RViz displays, tools and panel (see plugins_description.xml)
add_library(${PROJECT_NAME}_rviz_plugins SHARED
  src/ForbiddenZoneTool.cpp include/ugv_nav4d_ros2/ForbiddenZoneTool.hpp
  src/Goal3DTool.cpp include/ugv_nav4d_ros2/Goal3DTool.hpp
  src/InspectOrientationsTool.cpp include/ugv_nav4d_ros2/InspectOrientationsTool.hpp
  src/InspectTraversabilityTool.cpp include/ugv_nav4d_ros2/InspectTraversabilityTool.hpp
  src/JumpToRobotTool.cpp include/ugv_nav4d_ros2/JumpToRobotTool.hpp
  src/MLSMapDisplay.cpp include/ugv_nav4d_ros2/MLSMapDisplay.hpp
  src/OperatorPanel.cpp include/ugv_nav4d_ros2/OperatorPanel.hpp
  src/TravMapDisplay.cpp include/ugv_nav4d_ros2/TravMapDisplay.hpp
  src/Waypoint3DTool.cpp include/ugv_nav4d_ros2/Waypoint3DTool.hpp
)
set_target_properties(${PROJECT_NAME}_rviz_plugins PROPERTIES AUTOMOC ON)
target_include_directories(${PROJECT_NAME}_rviz_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_rviz_plugins PRIVATE
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  pluginlib::pluginlib
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
  Qt5::Widgets
  Eigen3::Eigen
  PkgConfig::ugv_nav4d
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ugv_nav4d_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES icons/classes/MultiLayeredSurfaceMap.png
              icons/classes/TraversabilityMap.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

install(FILES plugins_description.xml
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/visualize_path.py
  scripts/follow_path_client.py
  scripts/bag_recorder.py
  scripts/field_operations.py
  scripts/test_footprint_adaptation.py
  scripts/posture_publisher.py
  scripts/pc2_to_ply.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()

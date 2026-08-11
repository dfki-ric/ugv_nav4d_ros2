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
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(rosidl_default_generators REQUIRED)
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

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Eigen3
  rclcpp
  rclcpp_components
  tf2_geometry_msgs
  std_msgs
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  tf2_ros
  nav_msgs
  visualization_msgs
  pcl_conversions
  rviz_common
  rviz_rendering
  pluginlib
  std_srvs
  controller_manager_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
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
  "srv/InspectTraversability.srv"
  "srv/MissionFile.srv"
  "action/SaveMLSMap.action"
  DEPENDENCIES geometry_msgs std_msgs nav_msgs builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  "ugv_nav4d_ros2_msgs" "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME} SHARED src/ugv_nav4d_ros2.cpp)
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${ugv_nav4d_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${ugv_nav4d_LDFLAGS}
  Eigen3::Eigen
)
# omp_set_num_threads(numThreads) in ugv_nav4d_ros2.cpp caps the traversability
# generator's wave-parallel expansion with the SAME numThreads parameter the
# planner uses. Without OpenMP the call compiles out (and travgen runs serial).
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

qt5_wrap_cpp(MOC_TRAV_MAP_FILES include/ugv_nav4d_ros2_trav_map_plugin/TravMapDisplay.hpp)
# Create the plugin library
add_library(ugv_nav4d_ros2_trav_map_plugin SHARED src/TravMapDisplay.cpp ${MOC_TRAV_MAP_FILES})
target_link_libraries(ugv_nav4d_ros2_trav_map_plugin "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_trav_map_plugin  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_trav_map_plugin PUBLIC 
  ${ugv_nav4d_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_trav_map_plugin/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_trav_map_plugin
  Eigen3::Eigen  
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_MLS_MAP_FILES include/ugv_nav4d_ros2_mls_map_plugin/MLSMapDisplay.hpp)
# Create the plugin library
add_library(ugv_nav4d_ros2_mls_map_plugin SHARED src/MLSMapDisplay.cpp ${MOC_MLS_MAP_FILES})
target_link_libraries(ugv_nav4d_ros2_mls_map_plugin "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_mls_map_plugin  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_mls_map_plugin PUBLIC 
  ${ugv_nav4d_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_mls_map_plugin/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_mls_map_plugin
  Eigen3::Eigen  
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_GOAL3D_TOOL_FILES include/ugv_nav4d_ros2_goal3d_tool/Goal3DTool.hpp)
# Create the tool plugin library
add_library(ugv_nav4d_ros2_goal3d_tool SHARED src/Goal3DTool.cpp ${MOC_GOAL3D_TOOL_FILES})
ament_target_dependencies(ugv_nav4d_ros2_goal3d_tool  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_goal3d_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_goal3d_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_goal3d_tool
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_FORBIDDEN_ZONE_TOOL_FILES include/ugv_nav4d_ros2_forbidden_zone_tool/ForbiddenZoneTool.hpp)
# Create the tool plugin library
add_library(ugv_nav4d_ros2_forbidden_zone_tool SHARED src/ForbiddenZoneTool.cpp ${MOC_FORBIDDEN_ZONE_TOOL_FILES})
target_link_libraries(ugv_nav4d_ros2_forbidden_zone_tool "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_forbidden_zone_tool  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_forbidden_zone_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_forbidden_zone_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_forbidden_zone_tool
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_INSPECT_ORIENTATIONS_TOOL_FILES include/ugv_nav4d_ros2_inspect_orientations_tool/InspectOrientationsTool.hpp)
# Create the orientation-inspection tool plugin library
add_library(ugv_nav4d_ros2_inspect_orientations_tool SHARED src/InspectOrientationsTool.cpp ${MOC_INSPECT_ORIENTATIONS_TOOL_FILES})
target_link_libraries(ugv_nav4d_ros2_inspect_orientations_tool "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_inspect_orientations_tool  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_inspect_orientations_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_inspect_orientations_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_inspect_orientations_tool
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_JUMP_TO_ROBOT_TOOL_FILES include/ugv_nav4d_ros2_jump_to_robot_tool/JumpToRobotTool.hpp)
# Create the jump-to-robot tool plugin library
add_library(ugv_nav4d_ros2_jump_to_robot_tool SHARED src/JumpToRobotTool.cpp ${MOC_JUMP_TO_ROBOT_TOOL_FILES})
ament_target_dependencies(ugv_nav4d_ros2_jump_to_robot_tool  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_jump_to_robot_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_jump_to_robot_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_jump_to_robot_tool
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_OPERATOR_PANEL_FILES include/ugv_nav4d_ros2_operator_panel/OperatorPanel.hpp)
# Create the panel plugin library
add_library(ugv_nav4d_ros2_operator_panel SHARED src/OperatorPanel.cpp ${MOC_OPERATOR_PANEL_FILES})
target_link_libraries(ugv_nav4d_ros2_operator_panel "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_operator_panel  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_operator_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_operator_panel/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_operator_panel
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_WAYPOINT3D_TOOL_FILES include/ugv_nav4d_ros2_waypoint3d_tool/Waypoint3DTool.hpp)
# Create the tool plugin library
add_library(ugv_nav4d_ros2_waypoint3d_tool SHARED src/Waypoint3DTool.cpp ${MOC_WAYPOINT3D_TOOL_FILES})
target_link_libraries(ugv_nav4d_ros2_waypoint3d_tool "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_waypoint3d_tool  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_waypoint3d_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_waypoint3d_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_waypoint3d_tool
  Qt5::Core
  Qt5::Widgets
)

qt5_wrap_cpp(MOC_INSPECT_TOOL_FILES include/ugv_nav4d_ros2_inspect_tool/InspectTraversabilityTool.hpp)
add_library(ugv_nav4d_ros2_inspect_tool SHARED src/InspectTraversabilityTool.cpp ${MOC_INSPECT_TOOL_FILES})
target_link_libraries(ugv_nav4d_ros2_inspect_tool "${cpp_typesupport_target}")
ament_target_dependencies(ugv_nav4d_ros2_inspect_tool ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_inspect_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_inspect_tool/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ugv_nav4d_ros2_inspect_tool Qt5::Core Qt5::Widgets)

add_executable(ugv_nav4d_ros2_node
	src/ugv_nav4d_ros2_node.cpp
)

target_link_libraries(ugv_nav4d_ros2_node PRIVATE
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_trav_map_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_mls_map_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_goal3d_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_waypoint3d_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_operator_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_forbidden_zone_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_inspect_orientations_tool
  ugv_nav4d_ros2_jump_to_robot_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ugv_nav4d_ros2_inspect_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()

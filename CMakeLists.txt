cmake_minimum_required(VERSION 3.8)
project(ugv_nav4d_ros2)

set(ugv_nav4d_ros2_MAJOR_VERSION 0)
set(ugv_nav4d_ros2_MINOR_VERSION 0)
set(ugv_nav4d_ros2_PATCH_VERSION 0)
set(ugv_nav4d_ros2_VERSION
  ${ugv_nav4d_ros2_MAJOR_VERSION}.${ugv_nav4d_ros2_MINOR_VERSION}.${ugv_nav4d_ros2_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencis
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ugv_nav4d-qt5 REQUIRED IMPORTED_TARGET ugv_nav4d-qt5)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Eigen3
  rclcpp
  rclcpp_components
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  nav_msgs
  pcl_conversions
  rviz_common
  rviz_rendering
  pluginlib
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces("ugv_nav4d_ros2_msgs"
  "msg/TravPatch.msg"
  "msg/TravMap.msg"
  DEPENDENCIES geometry_msgs std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED src/ugv_nav4d_ros2.cpp)
rosidl_target_interfaces(${PROJECT_NAME} "ugv_nav4d_ros2_msgs" "rosidl_typesupport_cpp")
ament_target_dependencies(${PROJECT_NAME}  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${ugv_nav4d-qt5_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${ugv_nav4d-qt5_LDFLAGS}
  Eigen3::Eigen
)

qt5_wrap_cpp(MOC_FILES include/ugv_nav4d_ros2_trav_map_plugin/TravMapDisplay.hpp)
# Create the plugin library
add_library(ugv_nav4d_ros2_trav_map_plugin SHARED src/TravMapDisplay.cpp ${MOC_FILES})
rosidl_target_interfaces(ugv_nav4d_ros2_trav_map_plugin "ugv_nav4d_ros2_msgs" "rosidl_typesupport_cpp")
ament_target_dependencies(ugv_nav4d_ros2_trav_map_plugin  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(ugv_nav4d_ros2_trav_map_plugin PUBLIC 
  ${ugv_nav4d-qt5_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ugv_nav4d_ros2_trav_map_plugin/>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen  
  Qt5::Core
  Qt5::Widgets
)

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

install(TARGETS ugv_nav4d_ros2_node 
	DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugins_description.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()

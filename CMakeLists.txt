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

# find dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(ugv_nav4d REQUIRED IMPORTED_TARGET ugv_nav4d)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ugv_nav4d
	src/ugv_nav4d.cpp
)

target_link_libraries(ugv_nav4d PRIVATE
    PkgConfig::ugv_nav4d
	  tf2_ros::tf2_ros
	  ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    rclcpp::rclcpp
)

install(
	TARGETS ugv_nav4d
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

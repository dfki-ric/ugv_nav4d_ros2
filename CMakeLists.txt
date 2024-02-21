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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

# Define the source files for your library
set(SOURCES
  src/ugv_nav4d.cpp
)

add_library(ugv_nav4d_ros2 SHARED ${SOURCES})

ament_target_dependencies(ugv_nav4d_ros2 geometry_msgs std_msgs rclcpp rclcpp_components tf2_ros tf2_geometry_msgs)

target_include_directories(ugv_nav4d_ros2 PUBLIC 
  ${ugv_nav4d_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ugv_nav4d_ros2
  ${ugv_nav4d_LDFLAGS}
  Eigen3::Eigen
)

add_executable(ugv_nav4d_node
	src/ugv_nav4d_node.cpp
)
target_link_libraries(ugv_nav4d_node PRIVATE
    ugv_nav4d_ros2
)

install(TARGETS ugv_nav4d_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
	TARGETS ugv_nav4d_node 
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)
#find_package(PCL 1.7 REQUIRED COMPONENTS io common)
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
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Define the source files for your library
set(SOURCES
  src/ugv_nav4d.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}  ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${ugv_nav4d-qt5_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${ugv_nav4d-qt5_LDFLAGS}
  Eigen3::Eigen
)

add_executable(ugv_nav4d_node
	src/ugv_nav4d_node.cpp
)
target_link_libraries(ugv_nav4d_node PRIVATE
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
	TARGETS ugv_nav4d_node 
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

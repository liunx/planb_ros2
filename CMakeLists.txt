cmake_minimum_required(VERSION 3.5)
project(planb_ros2)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# Planb policy node
#=============
add_library(
  policy_node SHARED
  src/policy_node.cpp
)

target_compile_definitions(
  policy_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(
  policy_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
)

rclcpp_components_register_nodes(policy_node "planb::PolicyNode")
set(node_plugins "${node_plugins}policy::PolicyNode;$<TARGET_FILE:policy_node>\n")

#=============
# Planb camera node
#=============
add_library(
  camera_node SHARED
  src/camera_node.cpp
)

target_compile_definitions(
  camera_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(
  camera_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
)

rclcpp_components_register_nodes(camera_node "planb::CameraNode")
set(node_plugins "${node_plugins}camera::CameraNode;$<TARGET_FILE:camera_node>\n")

#=============
# Planb policy main
#=============
add_executable(
  policy_main
  src/policy_main.cpp
)

target_link_libraries(
  policy_main
  policy_node
)

ament_target_dependencies(
  policy_main
  rclcpp
)

#=============
# Planb camera main
#=============
add_executable(
  camera
  src/camera.cpp
)

target_link_libraries(
  camera
  camera_node
)

ament_target_dependencies(
  camera
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_policy_node export_camera_node)

ament_export_libraries(policy_node camera_node)


#=============
# Install
#=============

# Install nodes

install(
  TARGETS policy_node
  EXPORT export_policy_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS camera_node
  EXPORT export_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  policy_main
  camera
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
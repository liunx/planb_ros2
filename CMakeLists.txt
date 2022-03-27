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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Aruco.msg"
  "msg/Box.msg"
  DEPENDENCIES
  std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

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
# Planb aruco node
#=============
add_library(
  aruco_node SHARED
  src/aruco_node.cpp
)

target_compile_definitions(
  aruco_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(
  aruco_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
)

rclcpp_components_register_nodes(aruco_node "planb::ArucoNode")
set(node_plugins "${node_plugins}aruco::ArucoNode;$<TARGET_FILE:aruco_node>\n")

rosidl_target_interfaces(aruco_node
${PROJECT_NAME} "rosidl_typesupport_cpp")

#=============
# Planb tracker node
#=============
add_library(
  tracker_node SHARED
  src/tracker_node.cpp
)

target_compile_definitions(
  tracker_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(
  tracker_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
)

rclcpp_components_register_nodes(tracker_node "planb::TrackerNode")
set(node_plugins "${node_plugins}tracker::TrackerNode;$<TARGET_FILE:tracker_node>\n")

rosidl_target_interfaces(tracker_node
${PROJECT_NAME} "rosidl_typesupport_cpp")

#=============
# Planb hardware node
#=============
add_library(
  hardware_node SHARED
  src/hardware_node.cpp
)

target_compile_definitions(
  hardware_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(
  hardware_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
)

rclcpp_components_register_nodes(hardware_node "planb::HardwareNode")
set(node_plugins "${node_plugins}hardware::HardwareNode;$<TARGET_FILE:hardware_node>\n")

rosidl_target_interfaces(hardware_node
${PROJECT_NAME} "rosidl_typesupport_cpp")

#=============
# Planb camera main
#=============
add_executable(camera src/camera.cpp)
target_link_libraries(camera camera_node)
ament_target_dependencies(
  camera
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)

#=============
# Planb aruco main
#=============
add_executable(aruco src/aruco.cpp)
target_link_libraries(aruco aruco_node)
ament_target_dependencies(
  aruco
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)

#=============
# Planb tracker main
#=============
add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker tracker_node)
ament_target_dependencies(
  tracker
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)

#=============
# Planb hardware main
#=============
add_executable(hardware src/hardware.cpp)
target_link_libraries(hardware hardware_node)
ament_target_dependencies(
  hardware
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)

#=============
# Planb brain main
#=============
add_executable(brain src/brain.cpp src/behaviortree_action.cpp) 
ament_target_dependencies(
  brain
  rclcpp
  OpenCV
  behaviortree_cpp_v3
  sensor_msgs
  std_msgs
)
rosidl_target_interfaces(brain
${PROJECT_NAME} "rosidl_typesupport_cpp")

#=============
# Planb monitor main
#=============
add_executable(monitor src/monitor.cpp) 
ament_target_dependencies(
  monitor
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
)
rosidl_target_interfaces(monitor
${PROJECT_NAME} "rosidl_typesupport_cpp")

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(
  export_camera_node
  export_aruco_node
  export_tracker_node
  export_hardware_node
)

ament_export_libraries(
  camera_node
  aruco_node
  tracker_node
  hardware_node
)


#=============
# Install
#=============

# Install nodes
# camera
install(
  TARGETS camera_node
  EXPORT export_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# aruco
install(
  TARGETS aruco_node
  EXPORT export_aruco_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# tracker
install(
  TARGETS tracker_node
  EXPORT export_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# hardware
install(
  TARGETS hardware_node
  EXPORT export_hardware_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  camera
  aruco
  tracker
  hardware
  brain
  monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
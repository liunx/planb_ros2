cmake_minimum_required(VERSION 3.5)
project(planb_cv)

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
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(planb_interfaces REQUIRED)
find_package(planb_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

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
  planb_interfaces
  planb_common
)

rclcpp_components_register_nodes(aruco_node "planb::ArucoNode")
set(node_plugins "${node_plugins}aruco::ArucoNode;$<TARGET_FILE:aruco_node>\n")

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
  planb_interfaces
  planb_common
)

rclcpp_components_register_nodes(tracker_node "planb::TrackerNode")
set(node_plugins "${node_plugins}tracker::TrackerNode;$<TARGET_FILE:tracker_node>\n")

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
  planb_interfaces
  planb_common
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
  planb_interfaces
  planb_common
)

#=============
# Planb stereoCV main
#=============
add_executable(stereoCV src/stereo_cv.cpp) 
ament_target_dependencies(
  stereoCV
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Planb calib_tool main
#=============
add_executable(calib_tool src/calib_tool.cpp)
ament_target_dependencies(
  calib_tool
  rclcpp
  OpenCV
  Eigen3
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Planb stereo_calib main
#=============
add_executable(stereo_calib src/stereo_calib.cpp)
ament_target_dependencies(
  stereo_calib
  rclcpp
  OpenCV
  Eigen3
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Planb checkboard_capture main
#=============
add_executable(checkboard_capture src/checkboard_capture.cpp) 
ament_target_dependencies(
  checkboard_capture
  rclcpp
  OpenCV
  Eigen3
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Planb movie3d main
#=============
add_executable(movie3d src/movie3d.cpp) 
ament_target_dependencies(
  movie3d
  rclcpp
  OpenCV
  Eigen3
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Planb calibrate main
#=============
add_executable(calibrate src/calibrate.cpp) 
ament_target_dependencies(
  calibrate
  rclcpp
  OpenCV
  sensor_msgs
  std_msgs
  planb_interfaces
  planb_common
)

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(
  export_aruco_node
  export_tracker_node
)

ament_export_libraries(
  aruco_node
  tracker_node
)


#=============
# Install
#=============

# Install nodes
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

# Install executables
install(TARGETS
  aruco
  tracker
  stereoCV
  calib_tool
  stereo_calib
  checkboard_capture
  calibrate
  movie3d
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
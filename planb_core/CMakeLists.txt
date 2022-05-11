cmake_minimum_required(VERSION 3.5)
project(planb_core)

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
find_package(planb_interfaces REQUIRED)
find_package(planb_common REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

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
  planb_interfaces
  planb_common
)

#=============
# Planb motion_controller main
#=============
add_executable(motion_controller src/motion_controller.cpp) 
ament_target_dependencies(
  motion_controller
  rclcpp
  OpenCV
  Eigen3
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
)

ament_export_libraries(
)


#=============
# Install
#=============

# Install executables
install(TARGETS
  brain
  motion_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
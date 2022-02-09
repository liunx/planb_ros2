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
# Planb vision node
#=============

add_library(
  vision_node SHARED
  src/vision_node.cpp
)
target_compile_definitions(
  vision_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(
  vision_node
  class_loader
  OpenCV
  rclcpp
  sensor_msgs
)
rclcpp_components_register_nodes(vision_node "planb::VisionNode")
set(node_plugins "${node_plugins}vision::VisionNode;$<TARGET_FILE:vision_node>\n")

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
# Planb vision main
#=============

add_executable(
  vision_main
  src/vision_main.cpp
)
target_link_libraries(
  vision_main
  vision_node
)
ament_target_dependencies(
  vision_main
  rclcpp
)

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_policy_node export_vision_node)

ament_export_libraries(policy_node vision_node)

ament_package()

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
  TARGETS vision_node
  EXPORT export_vision_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS policy_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS vision_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

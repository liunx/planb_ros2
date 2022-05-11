cmake_minimum_required(VERSION 3.5)
project(planb_test)

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
find_package(rosidl_default_generators REQUIRED)

# Package includes not needed for CMake >= 2.8.11
include_directories(
  include
  ${ros2_shared_INCLUDE_DIRS}
)

#=============
# Planb tests main
#=============
add_executable(tests src/tests.cpp) 
ament_target_dependencies(
  tests
  rclcpp
  OpenCV
  Eigen3
  sensor_msgs
  std_msgs
)
target_link_libraries(tests "libmraa.so")

#=============
# Install
#=============

# Install executables
install(TARGETS
  tests
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
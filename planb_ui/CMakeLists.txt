cmake_minimum_required(VERSION 3.8)
project(planb_ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(planb_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

file(GLOB PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h )
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

include_directories(
        include/
       ${CMAKE_CURRENT_BINARY_DIR}
)

qt5_wrap_ui(QT_UI_HPP ${PRO_FORM_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOURCES_CPP ${PRO_RESOURCES_DIR})
add_executable(${PROJECT_NAME}
        ${PRO_SOURCES_DIR}
        ${QT_RESOURCES_CPP}
        ${PRO_INCLUDE_DIR}
        ${QT_MOC_HPP}
        ${PRO_FORM_DIR}
)
ament_target_dependencies(
  planb_ui
  rclcpp
  sensor_msgs
  std_msgs
  planb_interfaces
)

target_link_libraries(planb_ui Qt5::Widgets)

target_include_directories(planb_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planb_ui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS planb_ui
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

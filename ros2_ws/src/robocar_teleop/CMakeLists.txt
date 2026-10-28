cmake_minimum_required(VERSION 3.8)
project(robocar_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robocar_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED IMPORTED_TARGET libevdev)

add_executable(robocar_teleop_node src/robocar_teleop_node.cpp)
target_compile_options(robocar_teleop_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robocar_teleop_node
  rclcpp::rclcpp
  ${ackermann_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${robocar_msgs_TARGETS}
  PkgConfig::LIBEVDEV)

install(TARGETS robocar_teleop_node DESTINATION lib/${PROJECT_NAME})

ament_package()

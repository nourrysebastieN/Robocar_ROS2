cmake_minimum_required(VERSION 3.8)
project(robocar_vesc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(wesc REQUIRED)

add_executable(robocar_vesc_node src/robocar_vesc_node.cpp)
target_compile_options(robocar_vesc_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robocar_vesc_node
  rclcpp::rclcpp
  ${ackermann_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${vesc_msgs_TARGETS}
  wesc::wesc)

install(TARGETS robocar_vesc_node DESTINATION lib/${PROJECT_NAME})

ament_package()

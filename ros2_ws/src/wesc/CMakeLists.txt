cmake_minimum_required(VERSION 3.8)
project(wesc C)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Bibliothèque statique, liée dans robocar_vesc (et dans RoboCar par le Makefile)
add_library(wesc STATIC src/crc16.c src/read.c src/tty.c src/write.c)
target_compile_definitions(wesc PRIVATE _DEFAULT_SOURCE)
target_compile_options(wesc PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(wesc PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wesc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wesc PUBLIC m)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS wesc EXPORT export_wesc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_wesc HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  enable_language(CXX)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wesc test/test_wesc.cpp)
  target_link_libraries(test_wesc wesc)
endif()

ament_package()

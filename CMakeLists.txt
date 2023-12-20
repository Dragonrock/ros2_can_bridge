cmake_minimum_required(VERSION 3.5)
project(can_to_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msg REQUIRED)

include_directories(include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
add_executable(live_converter src/can_to_ros.cpp)

target_link_libraries(live_converter dl -lboost_system)

ament_target_dependencies(live_converter rclcpp std_msgs can_msg)

install(TARGETS
  live_converter
  DESTINATION ${PROJECT_NAME})


ament_package()

cmake_minimum_required(VERSION 3.8)
project(can_to_ros)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(live_converter src/main.cpp src/can_to_ros.cpp src/can_to_ros.hpp)
ament_target_dependencies(live_converter rclcpp std_msgs)

install(TARGETS
  live_converter
  DESTINATION lib/${PROJECT_NAME}
  )



ament_package()

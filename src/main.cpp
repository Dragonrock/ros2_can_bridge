#include "can_to_ros.hpp"

int main(int argc, char *argv[]) {
    //ROS2 Humble
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CanToRos>());
    rclcpp::shutdown();
    return 0;
}



#ifndef CAN_TO_ROS_HPP
#define CAN_TO_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "/home/dragon/ros2_ws/install/can_msg/include/can_msg/can_msg/msg/frame.hpp"
#include "log.h"

class CanToRos : public rclcpp::Node {
public:
  CanToRos();

private:
  void readCanFrame();
  void translateRosMsg();
  void CanPublisher(const can_msg::msg::Frame::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<can_msg::msg::Frame>::SharedPtr subscriber_;
  int socket_zero, socket_one;
  std::stringstream topicname_receive;
  std::stringstream topicname_transmit;
  std::stringstream servername;
};

#endif  // CAN_TO_ROS_HPP

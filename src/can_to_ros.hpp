#ifndef CAN_TO_ROS_HPP
#define CAN_TO_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float32.hpp>
#include "/home/dragon/ros2_ws/install/can_msg/include/can_msg/can_msg/msg/frame.hpp"
#include "log.h"

class CanToRos : public rclcpp::Node {
public:
  CanToRos();

private:
  void readCanFrame();
  void translateRosMsg(const struct can_frame& frame);
  void CanPublisher(const can_msg::msg::Frame::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_int;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_float;

  rclcpp::Subscription<can_msg::msg::Frame>::SharedPtr subscriber_;
  int socket_zero, socket_one;    

  std::unordered_map<uint32_t, std::pair<std::string, std::string>> can_id_mapping;
  std::stringstream topicname_receive;
  std::stringstream topicname_transmit;
  std::stringstream servername;
};

#endif  // CAN_TO_ROS_HPP

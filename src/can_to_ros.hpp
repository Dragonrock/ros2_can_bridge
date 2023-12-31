#ifndef CAN_TO_ROS_HPP
#define CAN_TO_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float32.hpp>

class CanToRos : public rclcpp::Node {
public:
  CanToRos();

private:
  void readCanFrameOne();
  void readCanFrameZero();
  
  template<typename T>
  void translatetoCan(const T& msg, uint32_t can_id);
  
  
  void translateRosMsg(const struct can_frame& frame);
  
  template<typename T>
  void readRosTopics(const typename T::SharedPtr msg, const std::string& topic_name);

  int socket_zero, socket_one;    


  std::unordered_map<uint32_t, std::pair<std::string, std::string>> can_id_mapping;
  std::unordered_map<uint32_t, int> can_id_socket_mapping;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_pub;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_subscriber;

  
  std::stringstream topicname_receive;
  std::stringstream topicname_transmit;
  std::stringstream servername;
};

#endif  // CAN_TO_ROS_HPP

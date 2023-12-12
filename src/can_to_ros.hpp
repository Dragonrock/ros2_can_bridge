#ifndef CAN_TO_ROS_HPP
#define CAN_TO_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
 
class CanToRos : public rclcpp::Node {
public:
  CanToRos();

private:
  void readCanFrame();
  void translateRosMsg();
  
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;

  int socket_zero, socket_one;
};

#endif  // CAN_TO_ROS_HPP

#include "can_to_ros.hpp"

//SocketCan Framework
#include <linux/can.h>
#include <linux/can/raw.h>

//Linux syscalls
#include <sys/ioctl.h>

//Network Libraries
#include <net/if.h>
#include <sys/socket.h>

//Messages Needed
#include "std_msgs/msg/int32.hpp"

CanToRos::CanToRos() : Node("can_to_ros_node") {
  publisher_ = create_publisher<std_msgs::msg::Int32>("pub_name", 10);
  //subscriber_ = create_subscription<std_msgs::msg::Int32>("receiver", 10, std::bind(&CanToRos::topic_callback, this,_1)); 

  // Socket creating and binding to Can interface
  socket_zero = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  struct sockaddr_can addr; //socket address
  struct ifreq ifr; //struct to interact with network devices(linux low level)

  std::string interface_name = "can0"; 

  strcpy(ifr.ifr_name, interface_name.c_str());
  ioctl(socket_zero, SIOCGIFINDEX, &ifr); //Get the index number of a Linux network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(socket_zero, (struct sockaddr *)&addr, sizeof(addr));

  socket_one = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  interface_name = "can1"; 
  strcpy(ifr.ifr_name, interface_name.c_str());
  ioctl(socket_one, SIOCGIFINDEX, &ifr); //Get the index number of a Linux network interface
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(socket_zero, (struct sockaddr *)&addr, sizeof(addr));


  // Set up the CAN frame handling in a separate thread
  std::thread thread(&CanToRos::readCanFrame, this);
  thread.detach();
}

void CanToRos::readCanFrame() {
  while (rclcpp::ok()) {
    struct can_frame frame_zero,frame_one;
    int buffer_zero = read(socket_zero, &frame_zero, sizeof(struct can_frame));
    int buffer_one = read(socket_one, &frame_one, sizeof(struct can_frame));

    int nbytes_zero = sizeof(buffer_zero)
    int nbytes_one = sizeof(buffer_one)

    if (nbytes_zero > 0) {
      
      std_msgs::msg::Int32 test_msg;

      zero_canID = std:hex<<can


     // publisher_->publish(test_msg);
    }

    if (nbytes_one > 0) {
      std_msgs::msg::Int32 test_msg2;
      //publisher_->publish(test_msg2);
    }
  }

  // Clean up
  close(socket_one);
  close(socket_zero);
}

void CanToRos::translateRosMsg(){
  ;
}

//void CanToRos::topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
//   {
//      RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg->data.c_str());
//    }
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
//; 
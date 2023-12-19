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
#include "/home/dragon/ros2_ws/install/can_msg/include/can_msg/can_msg/msg/frame.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

CanToRos::CanToRos() : Node("can_to_ros_node") {

  topicname_receive 	<< "CAN/" << "can0" << "/" << "receive";
  topicname_transmit 	<< "CAN/" << "can0" << "/" << "transmit";

  publisher_ = this->create_publisher<std_msgs::msg::Int32>("pub_name", 10);
  subscriber_ = this->create_subscription<can_msg::msg::Frame>(
    topicname_transmit, std::bind(&CanToRos::CanPublisher, this, _1));
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

  std::cout << "ROS2 to CAN-Bus topic:" << subscriber_->get_topic_name() 	<< std::endl;
  std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;

  // Set up the CAN frame handling in a separate thread
  std::thread thread(&CanToRos::readCanFrame, this);
  thread.detach();
}

void CanToRos::readCanFrame() {
  while (rclcpp::ok()) {
    struct can_frame frame_zero,frame_one;
    int buffer_zero = read(socket_zero, &frame_zero, sizeof(struct can_frame));
    int buffer_one = read(socket_one, &frame_one, sizeof(struct can_frame));

    int nbytes_zero = sizeof(buffer_zero);
    int nbytes_one = sizeof(buffer_one);

    if (nbytes_zero > 0) {
      
      std_msgs::msg::Int32 test_msg;

     // zero_canID = std::hex<<can


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

void CanToRos::CanPublisher(const can_msg::msg::Frame::SharedPtr msg) {    
    can_msg::msg::Frame msg1;
    msg1.id  = msg->id;
    msg1.dlc = msg->dlc;
    msg1.eff = msg->eff;
    msg1.rtr = msg->rtr;
    msg1.err = msg->err;
    msg1.data= msg->data;
}

void CanToRos::translateRosMsg(){
}

//void CanToRos::topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
//   {
//      RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg->data.c_str());
//    }
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
//; 



int main(int argc, char *argv[]) {
    //ROS2 Humble
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CanToRos>());
    rclcpp::shutdown();
    return 0;
}


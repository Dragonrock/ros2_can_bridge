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
  subscriber_ = this->create_subscription<can_msg::msg::Frame>(topicname_transmit.str(), rclcpp::QoS(10), std::bind(&CanToRos::CanPublisher, this, _1));
  // Socket creating and binding to Can interface
  socket_zero = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  struct sockaddr_can addr; //socket address
  struct ifreq ifr; //struct to interact with network devices(linux low level)

  std::string interface_name = "vcan0"; 

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
  bind(socket_one, (struct sockaddr *)&addr, sizeof(addr));

  std::cout << "ROS2 to CAN-Bus topic:" << subscriber_->get_topic_name() 	<< std::endl;
  std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name() 	<< std::endl;

  // Set up the CAN frame handling in a separate thread
  std::thread thread(&CanToRos::readCanFrame, this);
  thread.detach();
}

void CanToRos::readCanFrame() {
    while (rclcpp::ok()) {
        struct can_frame frame_zero, frame_one;
        int buffer_zero = read(socket_zero, &frame_zero, sizeof(struct can_frame));
        int buffer_one = read(socket_one, &frame_one, sizeof(struct can_frame));

        int nbytes_zero = sizeof(buffer_zero);
        int nbytes_one = sizeof(buffer_one);

        if (nbytes_zero > 0) {
            // Extract relevant data from frame_zero and publish
            std_msgs::msg::Int32 test_msg;
            test_msg.data = static_cast<int32_t>(frame_zero.can_id);  // Example: Assuming can_id is of type int32_t
            publisher_->publish(test_msg);
        }

        if (nbytes_one > 0) {
            // Extract relevant data from frame_one and publish
            std_msgs::msg::Int32 test_msg2;
            test_msg2.data = static_cast<int32_t>(frame_one.can_id);  // Example: Assuming can_id is of type int32_t
            publisher_->publish(test_msg2);
        }
    }

  // Clean up
  close(socket_one);
  close(socket_zero);
}

void CanToRos::CanPublisher(const can_msg::msg::Frame::SharedPtr msg)
{
  // Access the fields of the can_msg::msg::Frame message
  uint32_t id = msg->id;
  
  // Convert std::array to std::vector
  std::vector<uint8_t> data(msg->data.begin(), msg->data.end());

  // Process the CAN frame data as needed
  // ...

  // Example: Print the CAN frame information
  RCLCPP_INFO(this->get_logger(), "Received CAN frame - ID: %u, Data: [%02X %02X %02X %02X %02X %02X %02X %02X]",
              id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
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


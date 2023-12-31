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
#include <unordered_map>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

CanToRos::CanToRos() : Node("can_to_ros_node") {
///EDIT FROM HERE

  can_id_mapping = {
  {0x100, {"int_pub", typeid(std_msgs::msg::Int32).name()}},
  {0x200, {"accelerometer", typeid(std_msgs::msg::Float32).name()}}
};

  can_id_socket_mapping = {
  {0x100, socket_zero},
  {0x200, socket_one}
  };

  std::string interface_name0 = "vcan0"; 
  std::string interface_name1 = "vcan1"; 

  int_pub = this->create_publisher<std_msgs::msg::Int32>("int_pub", 10);
  float_pub = this->create_publisher<std_msgs::msg::Float32>("accelerometer", 10);

  throttle_subscriber = this->create_subscription<std_msgs::msg::Float32>("throttle", 10, 
    [this](std_msgs::msg::Float32::SharedPtr msg) {
        this->readRosTopics<std_msgs::msg::Float32>(msg, "throttle");
    }
);
///EDIT UNTIL HERE

  // Socket creating and binding to Can interface
  socket_zero = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  int loopback = 0;  // Disable loopback
  setsockopt(socket_zero, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  struct sockaddr_can addr; //socket address
  struct ifreq ifr; //struct to interact with network devices(linux low level)

  strcpy(ifr.ifr_name, interface_name0.c_str());
  ioctl(socket_zero, SIOCGIFINDEX, &ifr); //Get the index number of a Linux network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(socket_zero, (struct sockaddr *)&addr, sizeof(addr));

  socket_one = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  setsockopt(socket_one, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  strcpy(ifr.ifr_name, interface_name1.c_str());
  ioctl(socket_one, SIOCGIFINDEX, &ifr); //Get the index number of a Linux network interface
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(socket_one, (struct sockaddr *)&addr, sizeof(addr));


  std::thread thread_zero(&CanToRos::readCanFrameZero, this);
  thread_zero.detach();

  std::thread thread_one(&CanToRos::readCanFrameOne, this);
  thread_one.detach();
}

void CanToRos::readCanFrameZero() {
    while (rclcpp::ok()) {
        struct can_frame frame_zero;
        int buffer_zero = read(socket_zero, &frame_zero, sizeof(struct can_frame));
        int nbytes_zero = sizeof(buffer_zero);
        
        if (nbytes_zero > 0) {
          translateRosMsg(frame_zero);
        }
    }

    // Clean up
    close(socket_zero);
}

void CanToRos::readCanFrameOne() {
    while (rclcpp::ok()) {
        struct can_frame frame_one;
        int buffer_one = read(socket_one, &frame_one, sizeof(struct can_frame));
        int nbytes_one = sizeof(buffer_one);

        if (nbytes_one > 0) {
          translateRosMsg(frame_one);
        }
    }

    // Clean up
    close(socket_one);
}

void CanToRos::translateRosMsg(const struct can_frame& frame) {
    auto mapping_iter = can_id_mapping.find(frame.can_id);
    if (mapping_iter != can_id_mapping.end()) {
        const auto& mapping = mapping_iter->second;

        // Determine the message type based on the type name
        if (mapping.second == typeid(std_msgs::msg::Float32).name()) {
            std_msgs::msg::Float32 msg;
            // Convert frame.data to float
            float data;
            std::memcpy(&data, frame.data, sizeof(data));
            msg.data = data;

            // Use the correct publisher to publish the message
            float_pub->publish(msg);

        } else if (mapping.second == typeid(std_msgs::msg::Int32).name()) {
            std_msgs::msg::Int32 msg;
            // Convert frame.data to int
            int data;
            std::memcpy(&data, frame.data, sizeof(data));
            msg.data = data;

            // Use the correct publisher to publish the message
            int_pub->publish(msg);
        }
        // Add more else if statements for other message types
    }
}

// Function to read ROS topics and translate the data to CAN frames based on mappings
template<typename T>
void CanToRos::readRosTopics(const typename T::SharedPtr msg, const std::string& topic) {
  // Find the corresponding CAN ID for the topic
  uint32_t can_id = 0;
  for (const auto& pair : can_id_mapping) {
    if (pair.second.second == topic) {
        can_id = pair.first;
        break;
    }
  }

  if (can_id != 0) {
    // Translate the message to a CAN frame and send it
    translatetoCan(msg, can_id);
  }
}


template<typename T>
void CanToRos::translatetoCan(const T& msg, uint32_t can_id) {
  // Create a CAN frame with the specified ID and the data from the ROS message
  struct can_frame frame;
  frame.can_id = can_id;
  frame.can_dlc = sizeof(msg->data);
  memcpy(frame.data, &msg->data, sizeof(msg->data));

  // Find the CAN socket associated with the topic
  auto socket_iter = can_id_socket_mapping.find(can_id);
  if (socket_iter != can_id_socket_mapping.end()) {
    const int& socket = socket_iter->second;

    // Send the CAN frame to the socket
    write(socket, &frame, sizeof(struct can_frame));
  } else {
    // Handle the case where the socket for the CAN ID is not found
    RCLCPP_ERROR(get_logger(), "Socket for CAN ID '%d' not found", can_id);
  }
}


int main(int argc, char *argv[]) {
    //ROS2 Humble
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CanToRos>());
    rclcpp::shutdown();
    return 0;
}


***ROS 2/CAN Bridge***

**Description**

A ROS2 package enabling seamless communication between ROS 2 topics and a CAN bus. This bridge facilitates the translation of ROS 2 messages into CAN frames and vice-versa, allowing you to interact with CAN-based hardware within the ROS 2 ecosystem.

**Features**

* **Bidirectional Communication:** Translates ROS 2 messages to CAN frames and CAN frames to ROS 2 messages.
* **Message Mapping:** Offers flexible configuration for mapping ROS 2 topics to specific CAN IDs.
* **Multiple Socket Support:** Supports multiple CAN sockets to interface with different CAN buses or devices.

**Dependencies**

* ROS 2 (Distribution of your choice)
* SocketCAN libraries

**Installation**

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd <your_ros2_workspace>/src
   git clone https://github.com/Dragonrock/ros2can_bridge

2. Build your workspace:
    ```bash

    cd <your_ros2_workspace>
    colcon build

3. Source your workspace setup:
   ```bash
    source <your_ros2_workspace>/install/setup.bash

**Usage**
   1. Launch the bridge:
      ```bash
      ros2 run ros2can_bridge can_to_ros_node

   2. Customize Mappings:
    The can_to_ros.hpp file contains mappings for translating between ROS 2 topics and CAN IDs. Edit this file to adjust mappings according to your requirements.


**Explanation of Key Code Sections**

   * Mapping Dictionaries:
        * can_id_mapping: Maps CAN IDs to ROS 2 topics and their corresponding message types.
        * can_id_socket_mapping: Maps CAN IDs to specific CAN sockets, allowing for management of multiple CAN interfaces.

   * readCanFrameZero( ) and readCanFrameOne():
        * Functions that run in separate threads, each reading data from a dedicated CAN socket.
        * Incoming CAN frames are processed and translated to ROS 2 messages by the translateRosMsg function.

   * translateRosMsg( ):
        * Translates CAN frames into their corresponding ROS 2 messages based on the can_id_mapping.
        * Publishes the translated messages onto the appropriate ROS 2 topics.

   * readRosTopics ( ):
        * Subscribes to ROS 2 topics.
        * Calls the translatetoCan function to convert ROS 2 messages back into CAN frames.

   * translatetoCan( ):
        * Converts ROS 2 messages into CAN frames using the can_id_mapping.
        * Sends the CAN frame on the appropriate socket based on the can_id_socket_mapping.


*License*

This project is licensed under the MIT License

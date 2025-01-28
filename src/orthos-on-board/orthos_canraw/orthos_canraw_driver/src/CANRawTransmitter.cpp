#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <stdexcept>

// Global variables
int can_socket;
struct sockaddr_can can_addr;
can_msgs::Frame received_can_msg;

// Callback function for the "example" topic
void canMessageCallback(const can_msgs::Frame::ConstPtr& msg) {
    // Publish the received CAN message to the CAN bus
    struct can_frame frame;
    if ((msg->id == 0x06) || (msg->id == 0x05)) {
        // Science Payload and Chassis Lighting take standard frames.
        frame.can_id = msg->id;
    } else {
        frame.can_id = msg->id | CAN_EFF_FLAG;
    }
    frame.can_dlc = msg->dlc;
    std::memcpy(frame.data, msg->data.data(), msg->dlc);

    if (sendto(can_socket, &frame, sizeof(frame), 0, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0) {
        ROS_ERROR("Failed to send CAN frame to CAN bus.");
    } else {

    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "can_publisher_node");
    ros::NodeHandle nh;

    // Open a socket for CAN communication
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        ROS_FATAL("Failed to open CAN socket.");
        return -1;
    }

    // Specify the CAN interface (can1 in this case)
    std::string can_interface = "can1";

    // Initialize the CAN interface
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, can_interface.c_str());

    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        ROS_FATAL("Failed to get CAN interface index.");
        return -1;
    }

    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = ifr.ifr_ifindex;

    // Bind the socket to the CAN interface
    if (bind(can_socket, (struct sockaddr*)&can_addr, sizeof(can_addr)) < 0) {
        ROS_FATAL("Failed to bind CAN socket.");
        return -1;
    }

    // Create a subscribers for CAN topics
    // Arm Cubemars
    ros::Subscriber base_subscriber = nh.subscribe("can_msgs/orthos_arm/control/control_base", 10, canMessageCallback);
    ros::Subscriber elbw_subscriber = nh.subscribe("can_msgs/orthos_arm/control/control_elbw", 10, canMessageCallback);
    ros::Subscriber wrst_subscriber = nh.subscribe("can_msgs/orthos_arm/control/control_wrst", 10, canMessageCallback);
    // Swivel mount
    ros::Subscriber swivel_subscriber = nh.subscribe("can_msgs/orthos_swivel/control", 10, canMessageCallback);
    // Wrist
    ros::Subscriber wrist_subscriber = nh.subscribe("can_msgs/orthos_wrist/control", 10, canMessageCallback);
    // Science Payload
    ros::Subscriber science_subscriber = nh.subscribe("can_msgs/orthos_science/control", 10, canMessageCallback);
    // Chassis Lighting
    ros::Subscriber lighting_subscriber = nh.subscribe("can_msgs/orthos_chassis_lighting/control", 10, canMessageCallback);

    ROS_INFO("CAN Publisher Node is running.");

    // Spin to process callbacks and keep the node running
    ros::spin();

    // Close the CAN socket when the node is killed
    close(can_socket);

    return 0;
}

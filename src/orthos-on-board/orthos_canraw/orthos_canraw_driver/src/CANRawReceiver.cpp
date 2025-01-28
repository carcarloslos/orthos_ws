#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <orthos_canraw_driver/CubeMarsFeedback.h>

// Global variables for CAN IDs

// CubeMars Motor Feedback
const int base_id = 0x2914;
const int elbw_id = 0x2916;
const int wrst_id = 0x2917;

// OCU Feedback
const int ocu_can_id_lo = 0x20;
const int ocu_can_id_hi = 0x3B;

// S.P.A.S.T.I. Feedback
const int sci_can_id_lo = 0x3C;
const int sci_can_id_hi = 0x43;

// Wrist Feedback
const int wst_can_id_lo = 0x44;
const int wst_can_id_hi = 0x4D;

// Swivel Feedback
const int swl_can_id_lo = 0x4E;
const int swl_can_id_hi = 0x58;

int can_socket;
int16_t position, speed, current;
int8_t temperature;
uint8_t error_code;
struct sockaddr_can can_addr;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "can_receiver_node");
    ros::NodeHandle nh;

    // Declare received_can_msg
    can_msgs::Frame received_can_msg;
    // Declare CubeMarsFeedback
    orthos_canraw_driver::CubeMarsFeedback feedback_msg;

    // Declare publishers for the CubeMarsFeedback messages
    ros::Publisher base_feedback_pub = nh.advertise<orthos_canraw_driver::CubeMarsFeedback>("can_msgs/orthos_arm/feedback/feedback_base", 10);
    ros::Publisher elbw_feedback_pub = nh.advertise<orthos_canraw_driver::CubeMarsFeedback>("can_msgs/orthos_arm/feedback/feedback_elbw", 10);
    ros::Publisher wrst_feedback_pub = nh.advertise<orthos_canraw_driver::CubeMarsFeedback>("can_msgs/orthos_arm/feedback/feedback_wrst", 10);

    // Declare publishers for other Feedback messages
    // Swivel mount
    ros::Publisher swivel_publisher       = nh.advertise<can_msgs::Frame>("can_msgs/orthos_swivel/feedback", 10);
    // Wrist
    ros::Publisher wrist_publisher        = nh.advertise<can_msgs::Frame>("can_msgs/orthos_wrist/feedback", 10);
    // Science Payload
    ros::Publisher science_publisher      = nh.advertise<can_msgs::Frame>("can_msgs/orthos_science/feedback", 10);
    // OCU
    ros::Publisher ocu_publisher          = nh.advertise<can_msgs::Frame>("can_msgs/orthos_ocu/feedback", 10);
    // Unrecognized frames
    ros::Publisher unrecognized_publisher = nh.advertise<can_msgs::Frame>("can_msgs/orthos_unrecognized/feedback", 10);

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

    // Receive and process CAN messages in a loop
    while (ros::ok()) {
        struct can_frame frame;
        ssize_t nbytes = recvfrom(can_socket, &frame, sizeof(frame), 0, nullptr, nullptr);

        if (nbytes < 0) {
            ROS_ERROR("Failed to receive CAN frame.");
            continue;
        }

        if (nbytes == sizeof(frame)) {
            // Use the global CAN ID variables (base_id, elbw_id, wrst_id, end_eff_id, swivel_id) for processing
            feedback_msg.header.stamp = ros::Time::now();

            uint16_t shifted = frame.can_id;

            switch (shifted) {
                case base_id:
                    // Process CAN message with CAN ID equal to base_id
                    position = (frame.data[0] << 8) | frame.data[1];
                    speed = (frame.data[2] << 8) | frame.data[3];
                    current = (frame.data[4] << 8) | frame.data[5];

                    feedback_msg.position = static_cast<float>(position) * 0.1f;
                    feedback_msg.speed = static_cast<float>(speed) * 10.0f;
                    feedback_msg.current = static_cast<float>(current) * 0.01f;

                    feedback_msg.temperature = frame.data[6];
                    feedback_msg.error_code = frame.data[7];

                    // Publish to the "feedback_base" topic in namespace "orthos_arm/feedback"
                    base_feedback_pub.publish(feedback_msg);
                    break;

                case elbw_id:
                    // Process CAN message with CAN ID equal to base_id
                    position = (frame.data[0] << 8) | frame.data[1];
                    speed = (frame.data[2] << 8) | frame.data[3];
                    current = (frame.data[4] << 8) | frame.data[5];

                    feedback_msg.position = static_cast<float>(position) * 0.1f;
                    feedback_msg.speed = static_cast<float>(speed) * 10.0f;
                    feedback_msg.current = static_cast<float>(current) * 0.01f;

                    feedback_msg.temperature = frame.data[6];
                    feedback_msg.error_code = frame.data[7];

                    // Publish to the "feedback_base" topic in namespace "orthos_arm/feedback"
                    elbw_feedback_pub.publish(feedback_msg);
                    break;

                case wrst_id:
                    // Process CAN message with CAN ID equal to base_id
                    position = (frame.data[0] << 8) | frame.data[1];
                    speed = (frame.data[2] << 8) | frame.data[3];
                    current = (frame.data[4] << 8) | frame.data[5];

                    feedback_msg.position = static_cast<float>(position) * 0.1f;
                    feedback_msg.speed = static_cast<float>(speed) * 10.0f;
                    feedback_msg.current = static_cast<float>(current) * 0.01f;

                    feedback_msg.temperature = frame.data[6];
                    feedback_msg.error_code = frame.data[7];

                    // Publish to the "feedback_base" topic in namespace "orthos_arm/feedback"
                    wrst_feedback_pub.publish(feedback_msg);
                    break;

                default:

                    received_can_msg.id = frame.can_id;
                    received_can_msg.dlc = frame.can_dlc;
                    for (int i = 0; i < frame.can_dlc; i++) {
                        received_can_msg.data[i] = frame.data[i];
                    }                    

                    if (shifted >= ocu_can_id_lo && shifted <= ocu_can_id_hi) { // Handle messages from the OCU
                        ocu_publisher.publish(received_can_msg);
                    } else if (shifted >= sci_can_id_lo && shifted <= sci_can_id_hi) { // Handle messages from the science payload
                        science_publisher.publish(received_can_msg);
                    } else if (shifted >= wst_can_id_lo && shifted <= wst_can_id_hi) { // Handle messages from the wrist
                        received_can_msg.id = shifted;
                        wrist_publisher.publish(received_can_msg);
                    } else if (shifted >= swl_can_id_lo && shifted <= swl_can_id_hi) { // Handle messages from the swivel mount
                        swivel_publisher.publish(received_can_msg);
                    } else { // Handle unknown CAN IDs or add additional cases as needed
                        ROS_INFO("Received a message with an unknown CAN ID: 0x%X", frame.can_id);
                        unrecognized_publisher.publish(received_can_msg);
                    }
                    break;
            }
        }
    }

    // Close the CAN socket when the node is killed
    close(can_socket);

    return 0;
}

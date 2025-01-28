#include "ChassisController.h"

ChassisController::ChassisController(uint8_t id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, ros::NodeHandle& nh)
: can_id(id), can_subscriber_topic(can_subscriber_topic), can_publisher_topic(can_publisher_topic), nh(nh){
    // Initialize variables
    chassis_state = {0, 0, 0, 0, 0, 0};

    // Initialize the CAN message subscriber with the specified topic name
    can_subscriber = nh.subscribe<can_msgs::Frame>(
        can_subscriber_topic, 10, &ChassisController::handleCanMessage, this);

    // Initialize the CAN message publisher
    can_publisher = nh.advertise<can_msgs::Frame>(can_publisher_topic, 10);
}

bool ChassisController::handleChassisCommand(orthos_chassis_lighting::ChassisCommand::Request& req,
                                        orthos_chassis_lighting::ChassisCommand::Response& res) {
    // Create a CAN frame message
    can_msgs::Frame can_msg;

    // Fill in the CAN message fields based on req.command_id, req.value, and req.relais
    can_msg.id = can_id;
    can_msg.dlc = 8;
    
    // Send the frame data on the publisher in the form of the CAN message data

    can_msg.data[0] = req.light_state;
    can_msg.data[1] = req.red;
    can_msg.data[2] = req.green;
    can_msg.data[3] = req.blue;
    can_msg.data[4] = req.max_bright;
    can_msg.data[5] = req.min_bright;

    can_publisher.publish(can_msg);

    // Set the success flag in the response
    res.success = true;

    return true;
}

void ChassisController::handleCanMessage(const can_msgs::Frame::ConstPtr& msg) {
    // Extract relevant information from the CAN message and update class variables
    // Example: Check msg->id and msg->data to update variables accordingly
    // ...
}

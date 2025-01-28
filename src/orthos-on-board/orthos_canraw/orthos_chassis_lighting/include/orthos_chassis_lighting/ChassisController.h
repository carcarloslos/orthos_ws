#ifndef CHASSISCONTROLLER_H
#define CHASSISCONTROLLER_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_chassis_lighting/ChassisCommand.h"

class ChassisController {
public:
    struct ChassisState {
        uint8_t state;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t max_bright;
        uint8_t min_bright;
    };

    // Constructor
    ChassisController(uint8_t id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, ros::NodeHandle& nh);

    // ROS Service callback function for handling ChassisCommand requests
    bool handleChassisCommand(orthos_chassis_lighting::ChassisCommand::Request& req,
                            orthos_chassis_lighting::ChassisCommand::Response& res);

    uint8_t can_id;

private:

    ChassisState chassis_state;

    const std::string& can_subscriber_topic;
    const std::string& can_publisher_topic;

    ros::NodeHandle nh;

    // ROS Subscriber for receiving CAN messages
    ros::Subscriber can_subscriber;

    // ROS Publisher for sending CAN messages
    ros::Publisher can_publisher;

    // Callback function for the CAN message subscriber
    void handleCanMessage(const can_msgs::Frame::ConstPtr& msg);
};

#endif // CHASSISCONTROLLER_H

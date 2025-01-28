#ifndef SCIENCECONTROLLER_H
#define SCIENCECONTROLLER_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_science/ScienceCommand.h"
#include <orthos_science/ScienceData.h>

class ScienceController {
public:
    struct ScienceData {
        float payload_voltage;
        float payload_current;
        float payload_temperature;
        float apd_voltage;
        float apd_temperature;
        uint32_t relative_light_units;
        float position_drum;
        float weight;
    };

    // Constructor 
    ScienceController(uint8_t control_id, uint8_t feedback_id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, const std::string& data_publisher_topic, ros::NodeHandle& nh);

    // ROS Service callback function for handling ScienceCommand requests
    bool handleScienceCommand(orthos_science::ScienceCommand::Request& req,
                            orthos_science::ScienceCommand::Response& res);

    uint8_t control_id;
    uint8_t feedback_id;

    // Callback function for the CAN message subscriber
    void handleCanMessage(const can_msgs::Frame::ConstPtr& msg);

private:

    ScienceData science_data;

    orthos_science::ScienceData data_msg;

    const std::string& can_subscriber_topic;
    const std::string& can_publisher_topic;
    const std::string& data_publisher_topic;

    ros::NodeHandle nh;

    // ROS Subscriber for receiving CAN messages
    ros::Subscriber can_subscriber;

    // ROS Publisher for sending CAN messages
    ros::Publisher can_publisher;

    // ROS Publisher for sending Science Data messages
    ros::Publisher data_publisher;

};

#endif // SCIENCECONTROLLER_H

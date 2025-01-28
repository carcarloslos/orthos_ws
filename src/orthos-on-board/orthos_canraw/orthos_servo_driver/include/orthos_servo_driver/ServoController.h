#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_servo_driver/ServoCommand.h"
#include <orthos_servo_driver/ServoControllerFeedback.h>

class ServoController {
public:

    struct ServoPos {
        float servo_1;
        float servo_2;
        float servo_3;
        float servo_4;
    };

    struct FeedbackData {
        float feedback_1;
        float feedback_2;
        float feedback_3;
    };

    // Constructor
    ServoController(uint8_t control_id, uint8_t feedback_id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, const std::string& feedback_publisher_topic, ros::NodeHandle& nh);

    // ROS Service callback function for handling ServoCommand requests
    bool handleServoCommand(orthos_servo_driver::ServoCommand::Request& req,
                            orthos_servo_driver::ServoCommand::Response& res);

    uint8_t control_id;
    uint8_t feedback_id;


    // Callback function for the CAN message subscriber
    void handleCanMessage(const can_msgs::Frame::ConstPtr& msg);


private:
    ServoPos command_position;
    ServoPos actual_position;
    FeedbackData feedback_data;

    orthos_servo_driver::ServoControllerFeedback feedback_message;

    const std::string& can_subscriber_topic;
    const std::string& can_publisher_topic;
    const std::string& feedback_publisher_topic;

    ros::NodeHandle nh;

    // ROS Subscriber for receiving CAN messages
    ros::Subscriber can_subscriber;

    // ROS Publisher for sending CAN messages
    ros::Publisher can_publisher;

    // ROS Publisher for sending ServoControllerFeedback messages
    ros::Publisher feedback_publisher;
};

#endif // SERVOCONTROLLER_H

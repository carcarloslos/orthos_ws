#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_servo_driver/ServoController.h"
#include "orthos_servo_driver/ServoCommand.h"

const uint8_t control_id =  0x0C;
const uint8_t feedback_id = 0x4E;

int main(int argc, char** argv) {
    ros::init(argc, argv, "swivel_controller_node");
    ros::NodeHandle nh;

    std::string feedback_publisher_topic = "information/orthos_swivel/feedback";
    std::string can_publisher_topic = "can_msgs/orthos_swivel/control";
    std::string can_subscriber_topic = "can_msgs/orthos_swivel/feedback";

    // Instantiate the ServoController class with the specified CAN topic names
    ServoController swivel_controller(control_id, feedback_id, can_subscriber_topic, can_publisher_topic, feedback_publisher_topic, nh); 

    // Create a service server to handle ServoCommand requests
    ros::ServiceServer servo_command_service = nh.advertiseService(
        "swivel_command", &ServoController::handleServoCommand, &swivel_controller);

    ros::Rate rate(10); // Adjust the publishing rate as needed

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

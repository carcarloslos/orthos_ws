#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_chassis_lighting/ChassisController.h"
#include "orthos_chassis_lighting/ChassisCommand.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_lighting_node");
    ros::NodeHandle nh;

    std::string can_publisher_topic = "can_msgs/orthos_chassis_lighting/control";
    std::string can_subscriber_topic = "can_msgs/orthos_chassis_lighting/feedback";

    // Instantiate the ServoController class with the specified CAN topic names
    ChassisController chassis_controller(0x05, can_subscriber_topic, can_publisher_topic, nh); 

    // Create a service server to handle ServoCommand requests
    ros::ServiceServer chassis_lighting_command_service = nh.advertiseService(
        "chassis_lighting_command", &ChassisController::handleChassisCommand, &chassis_controller);

    ros::Rate rate(10); // Adjust the publishing rate as needed

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

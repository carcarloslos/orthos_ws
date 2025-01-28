#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "orthos_science/ScienceController.h"
#include "orthos_science/ScienceController.h"

const int control_id = 0x06;
const int feedback_id = 0x3C;

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_controller_node");
    ros::NodeHandle nh;

    std::string can_publisher_topic = "can_msgs/orthos_science/control";
    std::string can_subscriber_topic = "can_msgs/orthos_science/feedback";
    std::string data_publisher_topic = "information/orthos_science/data";

    // Instantiate the ServoController class with the specified CAN topic names
    ScienceController science_controller(control_id, feedback_id, can_subscriber_topic, can_publisher_topic, data_publisher_topic, nh);

    // Create a service server to handle ServoCommand requests
    ros::ServiceServer science_command_service = nh.advertiseService(
        "science_command", &ScienceController::handleScienceCommand, &science_controller);

    ros::Rate rate(10); // Adjust the publishing rate as needed

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

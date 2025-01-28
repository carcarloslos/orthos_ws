#include <ros/ros.h>
#include <orthos_science/ScienceCommand.h>
#include <cstdlib>  // For atoi

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_task_manager_node");
    ros::NodeHandle nh("~");

    std::string sequence;
    int tube_number = 1;  // Default value

    // Retrieve parameters
    nh.param<std::string>("sequence", sequence, "sequence1");
    nh.param<int>("tube_number", tube_number, 1);

    // Define sequences
    std::map<std::string, std::vector<std::tuple<uint8_t, float, uint8_t, uint8_t, double>>> sequences = {
        {"sequence1", {
            {5, 0.0, tube_number, 0, 0.0},
            {3, -25.0, 0, 0, 0.0}
        }},
        {"sequence2", {
            {3, -110.0, 0, 0, 0.0},
            {0, 20.0, 0, 0, 1.0},
            {3, 5.0, 0, 0, 1.0},
            {3, -110.0, 0, 0, 1.0},
            {3, 5.0, 0, 0, 1.0},
            {3, -110.0, 0, 0, 1.0},
            {3, 5.0, 0, 0, 0.0},
            {0, 0.0, 0, 0, 0.0}
        }},
        {"sequence3", {
            {5, 0.0,    tube_number, 2, 3.0},
            {0, 20.0,   0,           0, 5.0},
            {0, 0.0,    0,           0, 0.0},
            {5, 0.0,    tube_number, 1, 5.0},
            {9, 400.0,  0,           0, 5.0},
            {5, 0.0,    tube_number, 2, 3.0},
            {0, 20.0,   0,           0, 5.0},
            {0, 0.0,    0,           0, 0.0},
            {5, 0.0,    tube_number, 3, 3.0},
            {2, -115.0, 0,           0, 60.0},
            {2, -54.0,  0,           0, 0},
        }}
    };

    // Check if the provided sequence exists
    if (sequences.find(sequence) == sequences.end()) {
        ROS_ERROR("Invalid sequence parameter. Available sequences are: sequence1, sequence2, sequence3");
        return 1;
    }

    ros::ServiceClient service_client = nh.serviceClient<orthos_science::ScienceCommand>("/science_command");

    // Wait for the service to become available
    if (!service_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Service 'science_command' not available. Make sure it's running.");
        return 1;
    }

    // Execute the selected sequence
    for (const auto& command : sequences[sequence]) {
        orthos_science::ScienceCommand srv;
        srv.request.channel = std::get<0>(command);
        srv.request.value = std::get<1>(command);
        srv.request.tube = std::get<2>(command);
        srv.request.interface = std::get<3>(command);

        if (service_client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("Service call succeeded");
            } else {
                ROS_ERROR("Service call failed");
            }
        } else {
            ROS_ERROR("Failed to call service 'science_command'");
            return 1;
        }

        // Add a delay
        double delay = std::get<4>(command);
        ros::Duration(delay).sleep();
    }

    return 0;
}

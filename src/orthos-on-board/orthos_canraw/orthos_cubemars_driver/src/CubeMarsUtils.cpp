#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <orthos_cubemars_driver/CubeMarsFeedback.h>
#include "orthos_cubemars_driver/CubeMarsMotor.h"
#include <std_msgs/Bool.h> 
ros::ServiceClient wristCubeMarsClient; 
bool hall;
float wristposition;

// Set up the motors
CubeMarsMotor motor_base(0x14);
CubeMarsMotor motor_elbw(0x16);
CubeMarsMotor motor_wrst(0x17);

ros::Publisher base_pub; // Declare the publisher as a global variable
ros::Publisher elbw_pub; // Declare the publisher as a global variable
ros::Publisher wrst_pub; // Declare the publisher as a global variable
ros::Publisher wristhomed;


void hallcallback(const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id == 72) {
        hall = (msg->data[4] == 2 || msg->data[4] == 3);
    	ROS_INFO("Received boolhall %d", hall);
    }
}


// Callback function for the base feedback subscriber
void baseFeedbackCallback(const orthos_cubemars_driver::CubeMarsFeedback::ConstPtr& feedback_msg) {
    // Call the update function of the motor_base instance
    motor_base.UpdateFeedback(*feedback_msg);
}
// Callback function for the elbw feedback subscriber
void elbwFeedbackCallback(const orthos_cubemars_driver::CubeMarsFeedback::ConstPtr& feedback_msg) {
    // Call the update function of the motor_elbw instance
    motor_elbw.UpdateFeedback(*feedback_msg);
}
// Callback function for the wrst feedback subscriber

void wrstFeedbackCallback(const orthos_cubemars_driver::CubeMarsFeedback::ConstPtr& msg) {

    wristposition = msg->position;
    ROS_INFO("Received wrist %f", wristposition);

    }

/*
Set Origin Callbacks
*/
bool setBaseOriginCallback(orthos_cubemars_driver::SetOrigin::Request& req,
                          orthos_cubemars_driver::SetOrigin::Response& res) {
    // Create a CAN message
    can_msgs::Frame can_msg;

    motor_base.StopMotion();
    base_pub.publish(motor_base.cmd);
    
    // Fill in the CAN message fields as needed
    can_msg.id = motor_base.motor_id | ((uint32_t)5 << 8);
    can_msg.dlc = 8; // Data length code
    
    // Fill in the data field of the CAN message
    can_msg.data[0] = req.value;
    for (int i = 1; i < 8; ++i) {
        can_msg.data[i] = 0;
    }
    
    // Publish the CAN message
    base_pub.publish(can_msg);

    motor_base.StartMotionZero();
    base_pub.publish(motor_base.cmd);

    return true;
}

bool setElbwOriginCallback(orthos_cubemars_driver::SetOrigin::Request& req,
                          orthos_cubemars_driver::SetOrigin::Response& res) {
    // Create a CAN message
    can_msgs::Frame can_msg;

    motor_elbw.StopMotion();
    elbw_pub.publish(motor_elbw.cmd);
    
    // Fill in the CAN message fields
    can_msg.id = motor_elbw.motor_id | ((uint32_t)5 << 8);
    can_msg.dlc = 8; // Data length code
    
    // Fill in the data field of the CAN message
    can_msg.data[0] = req.value;
    for (int i = 1; i < 8; ++i) {
        can_msg.data[i] = 0;
    }
    
    // Publish the CAN message
    elbw_pub.publish(can_msg);

    motor_elbw.StartMotionZero();
    elbw_pub.publish(motor_elbw.cmd);

    return true;
}

bool setWrstHomingCallback(orthos_cubemars_driver::SetOrigin::Request& req,
                           orthos_cubemars_driver::SetOrigin::Response& res) {
ROS_INFO("Service /arm/home_wrist called with value: %d", req.value);

        if (hall == false) {
    	return false;
        }   
	orthos_cubemars_driver::CubeMarsPosVelGoal goal;
	goal.position_goal = -10;
	goal.speed_goal = 0;
	goal.accel_goal = 0;
	goal.control_mode = 0;

	while (hall == true) {
		ROS_INFO("goal.position_goal: %d", goal.position_goal);
      		motor_wrst.UpdateGoalState(goal);
    		motor_wrst.PositionVelocityCommand();
        }
        std_msgs::Bool boolmsg;
        boolmsg.data = true;
	wristhomed.publish(boolmsg);
	
	return true;
}


/*
State Goal Callbacks
*/
// Callback function for the base goal state subscriber
void baseGoalStateCallback(const orthos_cubemars_driver::CubeMarsPosVelGoal::ConstPtr& goal_msg) {
    // Update the goal state for motor_base
    motor_base.UpdateGoalState(*goal_msg);
    motor_base.PositionVelocityCommand();
}

// Callback function for the elbw goal state subscriber
void elbwGoalStateCallback(const orthos_cubemars_driver::CubeMarsPosVelGoal::ConstPtr& goal_msg) {
    // Update the goal state for motor_elbw
    motor_elbw.UpdateGoalState(*goal_msg);
    motor_elbw.PositionVelocityCommand();
}

// Callback function for the wrst goal state subscriber
void wrstGoalStateCallback(const orthos_cubemars_driver::CubeMarsPosVelGoal::ConstPtr& goal_msg) {
    // Update the goal state for motor_wrst
    motor_wrst.UpdateGoalState(*goal_msg);
    motor_wrst.PositionVelocityCommand();
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "can_msg_publisher_node");
    ros::NodeHandle nh;

    // Create publishers for the "elbw," "wrst," and "base" topics
    base_pub = nh.advertise<can_msgs::Frame>("can_msgs/orthos_arm/control/control_base", 10);
    elbw_pub = nh.advertise<can_msgs::Frame>("can_msgs/orthos_arm/control/control_elbw", 10);
    wrst_pub = nh.advertise<can_msgs::Frame>("can_msgs/orthos_arm/control/control_wrst", 10);


//----//Neue Publisher
    wristhomed = nh.advertise<std_msgs::Bool>("wrist_homed", 1000);

    // Create subscribers for the feedback topics
    ros::Subscriber base_feedback_sub = nh.subscribe<orthos_cubemars_driver::CubeMarsFeedback>(
        "can_msgs/orthos_arm/feedback/feedback_base", 10, baseFeedbackCallback);
    ros::Subscriber elbw_feedback_sub = nh.subscribe<orthos_cubemars_driver::CubeMarsFeedback>(
        "can_msgs/orthos_arm/feedback/feedback_elbw", 10, elbwFeedbackCallback);

    ros::Subscriber wristsub = nh.subscribe("/can_msgs/orthos_arm/feedback/feedback_wrst", 1000, wrstFeedbackCallback);
        
    // Create subscribers for the goal state topics
    ros::Subscriber base_goal_state_sub = nh.subscribe<orthos_cubemars_driver::CubeMarsPosVelGoal>(
        "arm/goal_state/base", 10, baseGoalStateCallback);
    ros::Subscriber elbw_goal_state_sub = nh.subscribe<orthos_cubemars_driver::CubeMarsPosVelGoal>(
        "arm/goal_state/elbw", 10, elbwGoalStateCallback);
    ros::Subscriber wrst_goal_state_sub = nh.subscribe<orthos_cubemars_driver::CubeMarsPosVelGoal>(
        "arm/goal_state/wrst", 10, wrstGoalStateCallback);
        
//----//Neue Subscriber
    ros::Subscriber hallsub = nh.subscribe("/can_msgs/orthos_wrist/feedback", 1000, hallcallback);
 
    
    // Set origin Services
    ros::ServiceServer set_base_origin_service = nh.advertiseService(
    "arm/set_origin/base", &setBaseOriginCallback);
    ros::ServiceServer set_elbw_origin_service = nh.advertiseService(
    "arm/home_elbw", &setElbwOriginCallback);
    ros::ServiceServer service = nh.advertiseService("arm/home_wrist", setWrstHomingCallback);

    // Set position Services
    ros::ServiceServer set_base_position_service = nh.advertiseService(
        "base_position_command", &CubeMarsMotor::UpdatePositionGoal, &motor_base);
    ros::ServiceServer set_elbw_position_service = nh.advertiseService(
        "elbw_position_command", &CubeMarsMotor::UpdatePositionGoal, &motor_elbw);
    ros::ServiceServer set_wrst_position_service = nh.advertiseService(
        "wrst_position_command", &CubeMarsMotor::UpdatePositionGoal, &motor_wrst);
//----//neue services
    wristCubeMarsClient = nh.serviceClient
    <orthos_cubemars_driver::SetPositionGoal>("/wrst_position_command");
    
    // Set the rate at which you want to publish the CAN message (e.g., 10 Hz)
    ros::Rate rate(30); // 10 Hz

    while (ros::ok()) {
        // Publish the CAN message to the "elbw," "wrst," and "base" topics
        base_pub.publish(motor_base.cmd);  
        elbw_pub.publish(motor_elbw.cmd);
        wrst_pub.publish(motor_wrst.cmd);

        ros::spinOnce();
        // Sleep to control the publishing rate
        rate.sleep();
    }

    return 0;
}

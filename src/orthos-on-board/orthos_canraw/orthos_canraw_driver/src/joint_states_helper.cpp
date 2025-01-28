#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <orthos_canraw_driver/CubeMarsFeedback.h>
#include <orthos_servo_driver/ServoControllerFeedback.h>
#include <std_msgs/Bool.h> 

ros::Publisher pub;
sensor_msgs::JointState joint_state_msg;
double wristpose;
double elbowpose;
double elbowoffset = 0;
double wristoffset = 0;

void wrist_homed(const std_msgs::Bool::ConstPtr& msg)
{
 	wristoffset = wristpose - 11.4;
}

void ellbow_homed(const std_msgs::Bool::ConstPtr& msg)
{
	 elbowoffset = elbowpose + 20;
} 

void swivelnodecallback(const orthos_servo_driver::ServoControllerFeedback::ConstPtr& msg)
{
	ROS_INFO("Received base");
	sensor_msgs::JointState swivel1_msg;
	sensor_msgs::JointState swivel2_msg;
	swivel1_msg.name.push_back("swivel_joint_2");

	double swivpose;
	swivpose = {msg->actual_servo_1} ;

	swivpose =  swivpose * 3.1413 / 180 ;

	swivel1_msg.position = {swivpose};

	pub.publish(swivel1_msg);
	swivel2_msg.name.push_back("swivel_joint_3");
	swivpose = {msg->actual_servo_2} ;

	swivpose =  swivpose * 3.1413 / 180 ;

	swivel2_msg.position = {swivpose};

	pub.publish(swivel2_msg);
}


void basecallback(const orthos_canraw_driver::CubeMarsFeedback::ConstPtr& msg)
{       
	sensor_msgs::JointState joint_state_msg_base;
	joint_state_msg_base.name.push_back("atalante_arm_base_joint");
	joint_state_msg_base.position = {msg->position} ;
	double grad = {msg->position} ;
	grad = grad * 3.1413 / 180 ;
	joint_state_msg_base.position = {grad};
	// Publish the received message
	pub.publish(joint_state_msg_base);
}
  
    
void wristcallback(const orthos_canraw_driver::CubeMarsFeedback::ConstPtr& msg)
{    
	sensor_msgs::JointState wrist_msg;
	wrist_msg.name.push_back("atalante_arm_wrist_joint");
	double wristcalc;
	wristpose = {msg->position} ;

	wristcalc = wristpose - wristoffset;
	wristcalc =  wristcalc * 3.1413 / 180 ;

	wrist_msg.position = {wristcalc};

	pub.publish(wrist_msg);
}


void elbowcallback(const orthos_canraw_driver::CubeMarsFeedback::ConstPtr& msg)
{    
	sensor_msgs::JointState joint_state_msg_elbow;
	joint_state_msg_elbow.name.push_back("atalante_arm_elbow_joint");
	double elbowcalc;
	elbowpose = {msg->position} ;

	elbowcalc = elbowpose - ellbowoffset;
	elbowcalc =  elbowcalc * 3.1413 / 180 ;

	joint_state_msg_elbow.position = {elbowcalc};

	pub.publish(joint_state_msg_elbow);
}


void wristplatzhalter2callback(const orthos_servo_driver::ServoControllerFeedback::ConstPtr& msg)
{
	float wrist_yaw = {msg->actual_servo_2} ;
	float wrist_nod = {msg->actual_servo_1} ;
	wrist_yaw = wrist_yaw * 3.1413 / 180 ;
	wrist_nod = wrist_nod * 3.1413 / 180 ;
	sensor_msgs::JointState joint_state_msg_wrist;
	joint_state_msg_wrist.name.push_back("atalante_arm_pitch_joint");
	joint_state_msg_wrist.position = {wrist_nod} ;
	pub.publish(joint_state_msg_wrist);
	sensor_msgs::JointState joint_state_msg_wrist1;
	joint_state_msg_wrist1.name.push_back("atalante_arm_roll_joint");
	joint_state_msg_wrist1.position = {wrist_yaw} ;
	// Publish the received message
	pub.publish(joint_state_msg_wrist1);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_state_friend");
	ros::NodeHandle n;    

	pub = n.advertise<sensor_msgs::JointState>("orthos_hardware_interface_joint_states", 1000);

	ros::Subscriber sub1 = n.subscribe("/information/orthos_swivel/feedback", 1000, swivelnodecallback);

	ros::Subscriber sub3 = n.subscribe("/can_msgs/orthos_arm/feedback/feedback_base", 1000, basecallback);
	ros::Subscriber sub4 = n.subscribe("/can_msgs/orthos_arm/feedback/feedback_elbw", 1000, elbowcallback);

	ros::Subscriber sub5 = n.subscribe("/can_msgs/orthos_arm/feedback/feedback_wrst", 1000, wristcallback);
	ros::Subscriber sub6 = n.subscribe("information/orthos_wrist/feedback", 1000, wristplatzhalter2callback);
	ros::Subscriber sub8 = n.subscribe("ellbow_homed", 1000, ellbow_homed);
	ros::Subscriber sub9 = n.subscribe("wrist_homed", 1000, wrist_homed);
	ros::Rate rate(20); 

	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

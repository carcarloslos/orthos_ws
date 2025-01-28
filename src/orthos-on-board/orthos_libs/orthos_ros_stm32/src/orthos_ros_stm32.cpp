#include <ros/ros.h>
//#include <SerialPort.h>
//#include <SerialStream.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <math.h>
#include "senden.h"

//using namespace LibSerial;
//TODO chmod ttyS0 automatisch beim Start vom Rover
/*
const unsigned char PWM_START_BYTE = 0b10000001;
const unsigned char COMMAND_BYTE = 0b10000010; //After that byte Servo commands follow
const unsigned char LED_BYTE = 0b10000100;
const unsigned char PWM_STOP_BYTE = 0b10000011;
*/
double servo_command_left_front = 0; //Initial value
double servo_command_right_front = 0;
double servo_command_left_rear = 0;
double servo_command_right_rear = 0;

std_msgs::Float64 msg1;
std_msgs::Float64 msg2;
std_msgs::Float64 msg5;
std_msgs::Float64 msg6;

void send_to_stm_left_front(const std_msgs::Float64& msg)
{
    if(msg != msg1)
    {
    	msg1=msg;
    	senden(0, msg.data);
    }
    
}

void send_to_stm_left_rear(const std_msgs::Float64& msg)
{
    if(msg != msg2)
    {
	msg2=msg;
    	senden(2, msg.data);
    }
}

void send_to_stm_right_front(const std_msgs::Float64& msg)
{
    if(msg != msg5)
    {
    	msg5=msg;
    	senden(1, msg.data);
    }
}

void send_to_stm_right_rear(const std_msgs::Float64& msg)
{
    if(msg != msg6)
    {
    	msg6=msg;
    	senden(3, msg.data);
    }
}

void info_d(const double d)
{
    ROS_INFO("\n%f\n", d);
}

uint8_t decimal_byte(double value) {
    return (int8_t)value;

}

uint8_t float_byte(double value) {
    return (uint8_t)((value - (float)decimal_byte(value) * 256.0f));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "orthos_stm_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    ros::Subscriber servo_command_right_front_sub = n.subscribe("orthos_position_controller_right_front/command", 1, send_to_stm_right_front);
    ros::Subscriber servo_command_right_rear_sub = n.subscribe("orthos_position_controller_right_rear/command", 1, send_to_stm_right_rear);
    ros::Subscriber servo_command_left_front_sub = n.subscribe("orthos_position_controller_left_front/command", 1, send_to_stm_left_front);
    ros::Subscriber servo_command_left_rear_sub = n.subscribe("orthos_position_controller_left_rear/command", 1, send_to_stm_left_rear);

   // ros::Publisher cmd_vel_output = n.advertise<sensor_msgs::Imu>("imu", 10);



   loop_rate.sleep();
   ros::spin();
   
    
}

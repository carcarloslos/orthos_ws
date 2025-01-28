#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <unistd.h>

//#define M_PI 3.14159265358979323846

const float L1 = 438; // Approximation of distance between front axis and middle axis in mm
const float L2 = 497; // Approximation of distance between middle axis and rear axis in mm
const float T = 487; // Trackwidth in mm

float angle = 0.0; // Gets input from user controller
float velocity = 0; // Gets input from user controller

int64_t heart_time; // Time stamp of the latest heartbeat message from the ground station to detect connection loss



void set_angles(const geometry_msgs::Twist &msg) {
    ROS_INFO("I heard: [%f]", msg.linear.x);
    velocity = msg.linear.x;
    angle = msg.angular.z * 0.523599; // +-1 to +-30 degrees in radians (0.5235)
    if (abs(msg.angular.z) > 1.4) {
        // Limit maximum size to prevent oversteering. 
        // This is the maximum angle which is allowed as input (meaning it is 1/0.523599 times the maximum angle
        // which is used in the 'angle' value
        angle = 1.4 * 0.523599 * (angle / abs(angle)); 
    }
}


// Callback function to update the heart time variable
void check_heartbeat(const std_msgs::Bool beat) {
	ros::Time current_time = ros::Time::now();
	heart_time = current_time.toSec();
	ROS_INFO("heart: %i", heart_time);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "orthos_ackerman_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    ros::Subscriber keyboard_input = n.subscribe("cmd_vel", 100, set_angles); // Subscribes the velocity commands
    ros::Subscriber heartbeat_input = n.subscribe("heartbeat_topic", 10, check_heartbeat); 

    ros::Publisher left_front_angle = n.advertise<std_msgs::Float64>("orthos_position_controller_left_front/command",1); // Publishes Steering command topics
    ros::Publisher left_rear_angle = n.advertise<std_msgs::Float64>("orthos_position_controller_left_rear/command", 1);
    ros::Publisher right_front_angle = n.advertise<std_msgs::Float64>("orthos_position_controller_right_front/command",1);
    ros::Publisher right_rear_angle = n.advertise<std_msgs::Float64>("orthos_position_controller_right_rear/command",1);
    ros::Publisher left_front_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_1/command", 1); // Publishes Drive command topics
    ros::Publisher right_front_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_2/command", 1);
    ros::Publisher left_middle_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_3/command", 1);
    ros::Publisher right_middle_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_4/command", 1);
    ros::Publisher left_rear_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_5/command", 1);
    ros::Publisher right_rear_velocity = n.advertise<std_msgs::Float64>("locomotion/orthos_velocity_controller_6/command", 1);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/wheel_odom", 1000); // Publishes odom topic from motor commands

    std_msgs::Float64 velocity_msg_left_1;
    std_msgs::Float64 velocity_msg_left_2;
    std_msgs::Float64 velocity_msg_left_3;
    std_msgs::Float64 velocity_msg_right_1;
    std_msgs::Float64 velocity_msg_right_2;
    std_msgs::Float64 velocity_msg_right_3;
    std_msgs::Float64 angle_msg;

    nav_msgs::Odometry odom;

    float radius_of_center;
    double radii[6];
    double velocities[6];
    
    double right_angle_direction = 1.0;
    double left_angle_direction = 1.0;
    double right_direction = 1.0;
    double left_direction = 1.0;
    
    double abs_radius;
    
    
    double x = 0.0; // x-coordinate
    double y = 0.0; // y-coordinate
    double th = 0.0; // roation about z

    ros::Time currentTime, lastTime;
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    tf::TransformBroadcaster odom_broadcaster;
  
    while (ros::ok()) {    	
        // If the time limit for the last received message has been surpassed, the rover stops moving
        ros::Time current_time = ros::Time::now();
        int64_t next_heart_time = current_time.toSec();
        ROS_INFO("current time: %i", next_heart_time);
        int difference = next_heart_time - heart_time;
        ROS_INFO("difference: %i", difference);
        if (difference > 2) {
            velocity = 0;
            angle = 0;
            ROS_INFO("setting angle and velocity to 0");
        }
        radius_of_center = L1 / tan(angle); // Radius of center of circular motion
        // Left side
        angle_msg.data = atan(L1 / (radius_of_center - T / 2));
        left_front_angle.publish(angle_msg);
        // Record the angle direction -> needed to check if we've hit rotating on the spot part
        if (angle != 0) {
            left_angle_direction = angle_msg.data/abs(angle_msg.data);
        } else {
            left_angle_direction = 1.0;
        }
        angle_msg.data = -atan(L2 / (radius_of_center - T / 2));
        left_rear_angle.publish(angle_msg);
        // Right side
        angle_msg.data = atan(L1 / (radius_of_center + T / 2));
        right_front_angle.publish(angle_msg);
        // Record the angle direction -> needed to check if we've hit rotating on the spot part
        if (angle != 0) {
            right_angle_direction = angle_msg.data/abs(angle_msg.data);
        } else {
            right_angle_direction = 1.0;
        }
        angle_msg.data = -atan(L2 / (radius_of_center + T / 2));
        right_rear_angle.publish(angle_msg);

        if (angle != 0.0) {
            // Calculate radii of individual wheels
            abs_radius = abs(radius_of_center);
            radii[0] = sqrt(pow(abs_radius + T / 2, 2) + pow(L1, 2)); // outside front
            radii[1] = sqrt(pow(abs_radius - T / 2, 2) + pow(L1, 2)); // inside front
            radii[2] = abs(abs_radius + T / 2);                       // outside middle
            radii[3] = abs(abs_radius - T / 2);                       // inside middle
            radii[4] = sqrt(pow(abs_radius + T / 2, 2) + pow(L2, 2)); // outside rear
            radii[5] = sqrt(pow(abs_radius - T / 2, 2) + pow(L2, 2)); // inside rear
            // Calculate corresponding velocities -> this ratio is the result of constant 
            // angular velocities for different radii
            velocities[0] = abs(velocity * (radii[0] / abs_radius)); // outside front
            velocities[1] = abs(velocity * (radii[1] / abs_radius)); // inside front
            velocities[2] = abs(velocity * (radii[2] / abs_radius)); // outside middle
            velocities[3] = abs(velocity * (radii[3] / abs_radius)); // inside middle
            velocities[4] = abs(velocity * (radii[4] / abs_radius)); // outside rear
            velocities[5] = abs(velocity * (radii[5] / abs_radius)); // inside rear
        }

        // The next if loop determines which direction each side has to go in
        if ((right_angle_direction != left_angle_direction) && (velocity != 0)) {
            // Rover has move command and is turning on the spot
            // Turning on the spot -> ls and rs need to move in opposite directions
            right_direction = (velocity/abs(velocity)) * (angle/abs(angle));
            left_direction = right_direction * (-1.0);
            // Reassign appropriate velocities: wheel on opposite sides should now have 
            // the same velocity (mangitude)
            velocities[1] = velocities[0]; // Front wheels
            velocities[3] = velocities[2]; // Middle wheels
            velocities[5] = velocities[4]; // Rear wheels
        } else if (velocity != 0) {
            // Rover is moving but is not turning on the spot
            // Both sides should move in the same direction, which is that of the input velocity
            right_direction = velocity/abs(velocity);
            left_direction = velocity/abs(velocity);
        } else {
            // Velocity is zero. We assign the value 1 to prevent NaN
            right_direction = 1.0;
            left_direction = 1.0;
        }
	
        // Next, we need to figure out which of the sides is the 'outside' and which is 'inside'
        // and then assign the velocities accordingly.
        if (angle > 0) {
            // turning in the positive direction -> right side is the outside
            //std::cout << "turning left, right is outside." << std::endl;
            velocity_msg_right_1.data = right_direction * velocities[0]; // outside front
            velocity_msg_right_2.data = right_direction * velocities[2]; // outside middle
            velocity_msg_right_3.data = right_direction * velocities[4]; // outside rear

            velocity_msg_left_1.data = left_direction * velocities[1]; // inside front
            velocity_msg_left_2.data = left_direction * velocities[3]; // inside middle
            velocity_msg_left_3.data = left_direction * velocities[5]; // inside rear
        } else if (angle < 0) {
            // turning in the negative direction -> left side is outside
            //std::cout << "turning right, left is outside." << std::endl;
            velocity_msg_right_1.data = right_direction * velocities[1]; // inside front
            velocity_msg_right_2.data = right_direction * velocities[3]; // inside middle
            velocity_msg_right_3.data = right_direction * velocities[5]; // inside rear

            velocity_msg_left_1.data = left_direction * velocities[0]; // outside front
            velocity_msg_left_2.data = left_direction * velocities[2]; // outside middle
            velocity_msg_left_3.data = left_direction * velocities[4]; // outside rear
        } else {
            // angle is 0 -> moving in straight line -> all values identical
            velocity_msg_right_1.data = velocity; // right front
            velocity_msg_right_2.data = velocity; // right middle
            velocity_msg_right_3.data = velocity; // right rear

            velocity_msg_left_1.data = velocity; // left front
            velocity_msg_left_2.data = velocity; // left middle
            velocity_msg_left_3.data = velocity; // left rear
        }

        // Now we publish the values
        left_front_velocity.publish(velocity_msg_left_1);
        left_middle_velocity.publish(velocity_msg_left_2);
        left_rear_velocity.publish(velocity_msg_left_3);
        
        right_front_velocity.publish(velocity_msg_right_1);
        right_middle_velocity.publish(velocity_msg_right_2);
        right_rear_velocity.publish(velocity_msg_right_3);

	//std::cout << "angle: " << angle << std::endl;

        // And print them to the command line 
        /*std::cout << "links 1 (vorne): "  << velocity_msg_left_1.data << std::endl;
        std::cout << "links 2 (mitte): "  << velocity_msg_left_2.data << std::endl;
        std::cout << "links 3 (hinten): " << velocity_msg_left_3.data << std::endl;

        std::cout << "right 1 (vorne): "  << velocity_msg_right_1.data << std::endl;
        std::cout << "right 2 (mitte): "  << velocity_msg_right_2.data << std::endl;
        std::cout << "right 3 (hinten): " << velocity_msg_right_3.data << std::endl;*/


        // since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        currentTime = ros::Time::now();

        th += velocity / L1 * tan(angle);
        x += velocity * cos(th);
        y += velocity * sin(th);


        lastTime = currentTime;

        loop_rate.sleep();
        ros::spinOnce();
    }
}

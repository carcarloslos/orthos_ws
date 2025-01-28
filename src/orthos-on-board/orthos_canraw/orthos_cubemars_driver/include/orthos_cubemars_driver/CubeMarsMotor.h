#ifndef CUBEMARSMOTOR_H
#define CUBEMARSMOTOR_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <orthos_cubemars_driver/CubeMarsFeedback.h>
#include <orthos_cubemars_driver/CubeMarsPosVelGoal.h>
#include <orthos_cubemars_driver/CubeMarsCurrentLoopMsg.h>
#include <orthos_cubemars_driver/SetOrigin.h>
#include <orthos_cubemars_driver/SetPositionGoal.h>

class CubeMarsMotor {
public:
    struct GoalState {
        float position;
        int16_t speed;
        int16_t accel;
        int control_mode;
    };

    struct ServoFeedback {
        float position;
        float speed;
        float current;
        uint8_t temperature;
        uint8_t error_code;
    };

    CubeMarsMotor(uint8_t id);
    void CurrentLoopModeCommand(const orthos_cubemars_driver::CubeMarsCurrentLoopMsg& current_cmd_msg);
    void PositionVelocityCommand();
    void UpdateFeedback(const orthos_cubemars_driver::CubeMarsFeedback& feedback_msg);
    void UpdateGoalState(const orthos_cubemars_driver::CubeMarsPosVelGoal& goal_msg);
    void StartMotionZero();
    void StopMotion();

    uint8_t motor_id;
    can_msgs::Frame cmd;

    // ROS Service callback function for handling position requests
    bool UpdatePositionGoal(orthos_cubemars_driver::SetPositionGoal::Request& req,
                            orthos_cubemars_driver::SetPositionGoal::Response& res);
                            

private:
    int32_t current_cmd;
    GoalState goal_state;
    ServoFeedback servo_feedback;
};

#endif // CUBEMARSMOTOR_H

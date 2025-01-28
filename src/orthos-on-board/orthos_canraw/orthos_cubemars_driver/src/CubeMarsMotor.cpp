#include "CubeMarsMotor.h"

CubeMarsMotor::CubeMarsMotor(uint8_t id) : motor_id(id) {
    // Initialize other variables if needed
    goal_state = {0.0, 0, 50, 6};
    servo_feedback = {0.0, 0.0, 0.0, 0, 0};
    current_cmd = 0;
    cmd.id = motor_id;
    cmd.dlc = 8; // Set the data length code as needed
}

void CubeMarsMotor::CurrentLoopModeCommand(const orthos_cubemars_driver::CubeMarsCurrentLoopMsg& current_cmd_msg) {
    
    // Assemble a current command message based on the setting for current
    current_cmd = current_cmd_msg.current_cmd;

    // motor id sets the motor, 1 defines current loop mode
    cmd.id = motor_id | ((uint32_t)1 << 8);
    cmd.dlc = 4;
    cmd.data[0] = current_cmd >> 24;
    cmd.data[1] = (current_cmd) >> 16;
    cmd.data[2] = (current_cmd) >> 8;
    cmd.data[3] = (current_cmd);

}

void CubeMarsMotor::PositionVelocityCommand() {
    // Send position velocity command based on the current goal state
    int32_t pos_i = (int32_t)(goal_state.position * 10000);

    cmd.id = motor_id | ((uint32_t)6 << 8);
    cmd.dlc = 8;
    cmd.data[0] = pos_i >> 24;
    cmd.data[1] = (pos_i) >> 16;
    cmd.data[2] = (pos_i) >> 8;
    cmd.data[3] = (pos_i);
    cmd.data[4] = goal_state.speed >> 8;
    cmd.data[5] = goal_state.speed;
    cmd.data[6] = goal_state.accel >> 8;
    cmd.data[7] = goal_state.accel;
}

void CubeMarsMotor::UpdateGoalState(const orthos_cubemars_driver::CubeMarsPosVelGoal& goal_msg) {
    goal_state.position = goal_msg.position_goal;
    goal_state.speed = goal_msg.speed_goal;
    goal_state.accel = goal_msg.accel_goal;
    goal_state.control_mode = goal_msg.control_mode;
}

void CubeMarsMotor::StopMotion() {
    goal_state.position = 0.0;
    goal_state.speed = 0;

    int32_t pos_i = (int32_t)(goal_state.position * 10000);

    cmd.id = motor_id | ((uint32_t)6 << 8);
    cmd.dlc = 8;
    cmd.data[0] = pos_i >> 24;
    cmd.data[1] = (pos_i) >> 16;
    cmd.data[2] = (pos_i) >> 8;
    cmd.data[3] = (pos_i);
    cmd.data[4] = goal_state.speed >> 8;
    cmd.data[5] = goal_state.speed;
    cmd.data[6] = goal_state.accel >> 8;
    cmd.data[7] = goal_state.accel;
}

void CubeMarsMotor::StartMotionZero() {
    goal_state.position = 5.0;
    goal_state.speed = 50;

    int32_t pos_i = (int32_t)(goal_state.position * 10000);

    cmd.id = motor_id | ((uint32_t)6 << 8);
    cmd.dlc = 8;
    cmd.data[0] = pos_i >> 24;
    cmd.data[1] = (pos_i) >> 16;
    cmd.data[2] = (pos_i) >> 8;
    cmd.data[3] = (pos_i);
    cmd.data[4] = goal_state.speed >> 8;
    cmd.data[5] = goal_state.speed;
    cmd.data[6] = goal_state.accel >> 8;
    cmd.data[7] = goal_state.accel;
}

bool CubeMarsMotor::UpdatePositionGoal(orthos_cubemars_driver::SetPositionGoal::Request& req,
                        orthos_cubemars_driver::SetPositionGoal::Response& res) {

    goal_state.position = req.value;

    int32_t pos_i = (int32_t)(goal_state.position * 10000);

    cmd.id = motor_id | ((uint32_t)6 << 8);
    cmd.dlc = 8;
    cmd.data[0] = pos_i >> 24;
    cmd.data[1] = (pos_i) >> 16;
    cmd.data[2] = (pos_i) >> 8;
    cmd.data[3] = (pos_i);
    cmd.data[4] = goal_state.speed >> 8;
    cmd.data[5] = goal_state.speed;
    cmd.data[6] = goal_state.accel >> 8;
    cmd.data[7] = goal_state.accel;

    res.success = true;
    return true;
}

void CubeMarsMotor::UpdateFeedback(const orthos_cubemars_driver::CubeMarsFeedback& feedback_msg) {
    // Assign the contents of the feedback message to the feedback struct
    servo_feedback.position = feedback_msg.position;
    servo_feedback.speed = feedback_msg.speed;
    servo_feedback.current = feedback_msg.current;
    servo_feedback.temperature = feedback_msg.temperature;
    servo_feedback.error_code = feedback_msg.error_code;

}

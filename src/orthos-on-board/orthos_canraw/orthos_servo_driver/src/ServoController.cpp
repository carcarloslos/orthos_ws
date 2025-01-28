#include "ServoController.h"

ServoController::ServoController(uint8_t control_id, uint8_t feedback_id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, const std::string& feedback_publisher_topic, ros::NodeHandle& nh)
: control_id(control_id), feedback_id(feedback_id), can_subscriber_topic(can_subscriber_topic), can_publisher_topic(can_publisher_topic), feedback_publisher_topic(feedback_publisher_topic), nh(nh){
    // Initialize variables
    command_position = {0.0, 0.0, 0.0, 0.0};
    actual_position = {0.0, 0.0, 0.0, 0.0};
    feedback_data = {0.0, 0.0, 0.0};

    feedback_message.planned_servo_1 = 0.0;
    feedback_message.planned_servo_2 = 0.0;
    feedback_message.planned_servo_3 = 0.0;
    feedback_message.planned_servo_4 = 0.0;
    feedback_message.actual_servo_1 = 0.0;
    feedback_message.actual_servo_2 = 0.0;
    feedback_message.actual_servo_3 = 0.0;
    feedback_message.actual_servo_4 = 0.0;
    feedback_message.feedback_1 = 0.0;
    feedback_message.feedback_2 = 0.0;
    feedback_message.feedback_3 = 0.0;

    // Initialize the CAN message subscriber with the specified topic name
    can_subscriber = nh.subscribe<can_msgs::Frame>(
        can_subscriber_topic, 10, &ServoController::handleCanMessage, this);

    // Initialize the CAN message publisher
    can_publisher = nh.advertise<can_msgs::Frame>(can_publisher_topic, 10);

    // Initialize the Feedback message publisher
    feedback_publisher = nh.advertise<orthos_servo_driver::ServoControllerFeedback>(feedback_publisher_topic, 10);
}

bool ServoController::handleServoCommand(orthos_servo_driver::ServoCommand::Request& req,
                                        orthos_servo_driver::ServoCommand::Response& res) {
    // Create a CAN frame message
    can_msgs::Frame can_msg;

    // Fill in the CAN message fields based on req.command_id, req.value, and req.relais
    can_msg.id = control_id;
    can_msg.dlc = 8;
    
    // Interpret the float value as an integer to store it in the CAN message data
    float value_to_send = req.value;
    int32_t value_as_int = *reinterpret_cast<int32_t*>(&value_to_send);

    can_msg.data[0] = req.command_id;

    // Fill data fields 1 through 4 with the bytes of the float value
    for (int i = 0; i < 4; ++i) {
        can_msg.data[i + 1] = (value_as_int >> (i * 8)) & 0xFF;
    }

    // Fill data field 5 with the relais value
    can_msg.data[5] = req.relais;

    can_publisher.publish(can_msg);

    // Set the success flag in the response
    res.success = true;

    return true;
}

void ServoController::handleCanMessage(const can_msgs::Frame::ConstPtr& msg) {

    // uint32_t timestamp = msg->data[0] << 24 | msg->data[1] << 16 | msg->data[2] << 8 | msg->data[3] << 0;
    // uint32_t value = msg->data[4] << 24 | msg->data[5] << 16 | msg->data[6] << 8 | msg->data[7] << 0;

    float f_value;
    union {
        u_char b[4];
        float fval;
    } u;
    // The first four bytes are the timestamp. The next four are the float data.
    // Store the float data bytes in the union
    // Set value for float
    for (int i = 4; i < 8; i++) {
	u.b[i-4] = msg->data[i];
    }
    // Set value for float
    f_value = u.fval;

    if (msg->id == feedback_id) {
        // Servo 1 Planned
        command_position.servo_1 = f_value;
        feedback_message.planned_servo_1 = f_value;
    } else if (msg->id == (feedback_id + 1)) {
        // Servo 1 Read
        actual_position.servo_1 = f_value;
        feedback_message.actual_servo_1 = f_value;
    } else if (msg->id == (feedback_id + 2)) {
        // Servo 2 Planned
        command_position.servo_2 = f_value;
        feedback_message.planned_servo_2 = f_value;
    } else if (msg->id == (feedback_id + 3)) {
        // Servo 2 Read
        actual_position.servo_2 = f_value;
        feedback_message.actual_servo_2 = f_value;
    } else if (msg->id == (feedback_id + 4)) {
        // Servo 3 Planned
        command_position.servo_3 = f_value;
        feedback_message.planned_servo_3 = f_value;
    } else if (msg->id == (feedback_id + 5)) {
        // Servo 3 Read
        actual_position.servo_3 = f_value;
        feedback_message.actual_servo_3 = f_value;
    } else if (msg->id == (feedback_id + 6)) {
        // Servo 4 Planned
        command_position.servo_4 = f_value;
        feedback_message.planned_servo_4 = f_value;
    } else if (msg->id == (feedback_id + 7)) {
        // Servo 4 Read
        actual_position.servo_4 = f_value;
        feedback_message.actual_servo_4 = f_value;
    } else if (msg->id == (feedback_id + 8)) {
        feedback_data.feedback_1 = f_value;
        feedback_message.feedback_1 = f_value;
    } else if (msg->id == (feedback_id + 9)) {
        feedback_data.feedback_2 = f_value;
        feedback_message.feedback_2 = f_value;
    } else if (msg->id == (feedback_id + 10)) {
        feedback_data.feedback_3 = f_value;
        feedback_message.feedback_3 = f_value;
    } else {

    }
    
    feedback_publisher.publish(feedback_message);

}

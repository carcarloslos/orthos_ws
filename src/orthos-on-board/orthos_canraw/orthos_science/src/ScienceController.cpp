#include "ScienceController.h"

ScienceController::ScienceController(uint8_t control_id, uint8_t feedback_id, const std::string& can_subscriber_topic, const std::string& can_publisher_topic, const std::string& data_publisher_topic, ros::NodeHandle& nh)
: control_id(control_id), feedback_id(feedback_id), can_subscriber_topic(can_subscriber_topic), can_publisher_topic(can_publisher_topic), data_publisher_topic(data_publisher_topic), nh(nh){
    
    // Initialize variables
    science_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0};
    data_msg.payload_voltage = 0.0;
    data_msg.payload_current = 0.0;
    data_msg.payload_temperature = 0.0;
    data_msg.apd_voltage = 0.0;
    data_msg.apd_temperature = 0.0;
    data_msg.relative_light_units = 0;
    data_msg.position_drum = 0.0;
    data_msg.weight = 0.0;

    // Initialize the CAN message subscriber with the specified topic name
    can_subscriber = nh.subscribe<can_msgs::Frame>(
        can_subscriber_topic, 10, &ScienceController::handleCanMessage, this);

    // Initialize the CAN message publisher
    can_publisher = nh.advertise<can_msgs::Frame>(can_publisher_topic, 10);

    // Initialize the CAN message publisher
    data_publisher = nh.advertise<orthos_science::ScienceData>(data_publisher_topic, 10);
}

bool ScienceController::handleScienceCommand(orthos_science::ScienceCommand::Request& req,
                                        orthos_science::ScienceCommand::Response& res) {
    // Create a CAN frame message
    can_msgs::Frame can_msg;

    // Fill in the CAN message fields based on req.command_id, req.value, and req.relais
    can_msg.id = control_id;
    can_msg.dlc = 8;
    
    // Interpret the float value as an integer to store it in the CAN message data
    float value_to_send = req.value;
    int32_t value_as_int = *reinterpret_cast<int32_t*>(&value_to_send);

    can_msg.data[0] = req.channel;

    // Fill data fields 1 through 4 with the bytes of the float value
    for (int i = 0; i < 4; ++i) {
        can_msg.data[i + 1] = (value_as_int >> (i * 8)) & 0xFF;
    }

    // Fill data field 5 with the tube value
    can_msg.data[5] = req.tube;

    // Fill data field 6 with the interface value
    can_msg.data[6] = req.interface;

    can_publisher.publish(can_msg);

    // Set the success flag in the response
    res.success = true;

    return true;
}

void ScienceController::handleCanMessage(const can_msgs::Frame::ConstPtr& msg) {

    if (msg->id == feedback_id + 5) {
        // uint32 value
        uint value;
        u_char lfb[4]; // last four bytes
        // Fill with the last four bytes, these are the values we care about
        for (int i = 4; i < 8; i++) {
            lfb[i - 4] = msg->data[i]; // values in u_char format
            //TODO decode implementation nach https://docs.google.com/spreadsheets/d/1eto0RJzP25h62CYVw9JnzgdoyUMkE7NtDCtczJteavs/edit#gid=0
        }
        memcpy(&value, &lfb, sizeof(value)); //https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes
        science_data.relative_light_units = value;
        data_msg.relative_light_units = value;
    } else {
        float value;
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
        value = u.fval;

        if (msg->id == feedback_id) {
            // payload_voltage
            science_data.payload_voltage = value;
            data_msg.payload_voltage = value;
        } else if (msg->id == (feedback_id + 1)) {
            // payload_current
            science_data.payload_current = value;
            data_msg.payload_current = value;            
        } else if (msg->id == (feedback_id + 2)) {
            // payload_temperature
            science_data.payload_temperature = value;
            data_msg.payload_temperature = value;            
        } else if (msg->id == (feedback_id + 3)) {
            // apd_voltage
            science_data.apd_voltage = value;
            data_msg.apd_voltage = value; 
        } else if (msg->id == (feedback_id + 4)) {
            // apd_temperature
            science_data.apd_temperature = value;
            data_msg.apd_temperature = value; 
        } else if (msg->id == (feedback_id + 6)) {
            // position_drum
            science_data.position_drum = value;
            data_msg.position_drum = value; 
        } else if (msg->id == (feedback_id + 7)) {
            // weight
            science_data.weight = value;
            data_msg.weight = value; 
        } else {

        }
    }
    
    data_publisher.publish(data_msg);

}

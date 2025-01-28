#include <ros/ros.h>
#include <orthos_ocu_driver/OcuReading.h>
#include <std_msgs/Int16.h>
#include <cstdio>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <bitset>

float ocu_data[34]; // nicht für jedes frame eine eigene Variable, einfach alle float dinger hier drin
int ocu_state; //Die beiden int frames
int apd_rlu;
int lfb;

int decode_can_frame_int(struct can_frame frame) {
    uint value;
    u_char lfb[4]; // last four bytes
    // Fill with the last four bytes, these are the values we care about
    for (int i = frame.can_dlc - 4; i < frame.can_dlc; i++) {
    	lfb[i - (frame.can_dlc - 4)] = frame.data[i]; // values in u_char format
        //TODO decode implementation nach https://docs.google.com/spreadsheets/d/1eto0RJzP25h62CYVw9JnzgdoyUMkE7NtDCtczJteavs/edit#gid=0
        // This isn't properly passed on yet.
    }
    memcpy(&value, &lfb, sizeof(value)); //https://stackoverflow.com/questions/3991478/building-a-32-bit-float-out-of-its-4-composite-bytes
    // TODO: Already able to read out the value, but it is not properly forwarded to the topic.
    return value;
}

float decode_can_frame_float(struct can_frame frame) {
    float value;
    union {
        u_char b[4];
        float fval;
    } u;
    // The first four bytes are the timestamp. The next four are the float data.
    // Store the float data bytes in the union
    for (int i = 4; i < frame.can_dlc; i++) {
	u.b[i-4] = frame.data[i];
    }
    // Set value for float
    value = u.fval;
    return value;
}

void read_ocu(const int s) {
    struct can_frame frame;
    int nbytes = read(s, &frame, sizeof(struct can_frame)); // Blocks until a frame is available
     printf("0x%03X [%d] ",frame.can_id, frame.can_dlc);
     std::cout << std::endl;
    if (frame.can_id == 32) ocu_state = decode_can_frame_int(frame); // ID32 ist der State als int
    else if (frame.can_id > 32 && frame.can_id < 65 && frame.can_id != 62) ocu_data[frame.can_id - 33] = decode_can_frame_float(frame); // first valid can_id is 33, so shift it to fill the array starting at 0
    else if (frame.can_id == 62) apd_rlu = decode_can_frame_int(frame);
    if (nbytes < 0) ROS_ERROR("Read empty OCU frame. Publishing old values!");
}

void interpret_ocu_state(struct can_frame frame) { //WIP interpreting ocu_state bits 

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "orthos_ocu_driver_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(250); // TODO Ist eine schnelle Frequenz gut oder müsste eigentlich je ein einzelner Node für jede frame_id existieren?

    ros::Publisher ocu_readings_pub = n.advertise<orthos_ocu_driver::OcuReading>("ocu_readings", 1); //CAN ID 32
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        ROS_ERROR("Socket creation failed");
        return 1;
    }

    strcpy(ifr.ifr_name, "can1" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ROS_ERROR("Bind failed");
        return 1;
    }

    //-boost::thread read_ocu_thread(read_ocu);

    orthos_ocu_driver::OcuReading ocu_frame;
    while (ros::ok()) {
    	read_ocu(s);
        ocu_frame.state_ocu = ocu_state;
        ocu_frame.apd_rlu = apd_rlu;
        ocu_frame.voltage_24v_channel = ocu_data[0];
        ocu_frame.voltage_12v_channel = ocu_data[1];
        ocu_frame.voltage_7_4v_channel = ocu_data[2];
        ocu_frame.current_drivetrain_left = ocu_data[3];
        ocu_frame.current_drivetrain_right = ocu_data[4];
        ocu_frame.current_robot_arm = ocu_data[5];
        ocu_frame.current_24v_aux = ocu_data[6];
        ocu_frame.current_main_comp = ocu_data[7];
        ocu_frame.current_payload = ocu_data[8];
        ocu_frame.current_12v_aux_1 = ocu_data[9];
        ocu_frame.current_12v_aux_2 = ocu_data[10];
        ocu_frame.current_steering_front_left = ocu_data[11];
        ocu_frame.current_steering_front_right = ocu_data[12];
        ocu_frame.current_steering_rear_left = ocu_data[13];
        ocu_frame.current_steering_rear_right = ocu_data[14];
        ocu_frame.temperature_ocu_board_24v = ocu_data[15];
        ocu_frame.temperature_ocu_board_12v = ocu_data[16];
        ocu_frame.temperature_ocu_board_7_4v = ocu_data[17];
        ocu_frame.cell_voltage_1 = ocu_data[18];
        ocu_frame.cell_voltage_2 = ocu_data[19];
        ocu_frame.cell_voltage_3 = ocu_data[20];
        ocu_frame.cell_voltage_4 = ocu_data[21];
        ocu_frame.cell_voltage_5 = ocu_data[22];
        ocu_frame.cell_voltage_6 = ocu_data[23];
        ocu_frame.voltage_science_payload = ocu_data[24];
        ocu_frame.current_science_payload = ocu_data[25];
        ocu_frame.temperature_science_payload = ocu_data[26];
        ocu_frame.apd_bias_voltage = ocu_data[27];
        ocu_frame.apd_temperature = ocu_data[28];
        ocu_frame.position_drum = ocu_data[30];
        ocu_frame.weight = ocu_data[31];
        ocu_readings_pub.publish(ocu_frame);
        loop_rate.sleep();
        ros::spinOnce();
    }

    if (close(s) < 0) { // Close socket
        perror("Close");
        return 1;
    }


}

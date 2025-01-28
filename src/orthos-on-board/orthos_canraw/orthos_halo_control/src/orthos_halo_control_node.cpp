#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <cstdio>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>

int state; // Global state variable, will be changed each time a value is send to the OCU
// State:	0 = Idle	1 = Working	2 = Working Auto.	3 = Waiting	4 = Warning	5 = Error	6 = Gay	7 = Party
const char* states[8] = {"idle", "working", "working-auto", "waiting", "warning", "error", "gay", "party"};

void change_state(const std_msgs::Int16 &msg) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    printf("State %d\n", msg.data);

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return;
    }

    strcpy(ifr.ifr_name, "can1");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror("Bind");
        return;
    }

    frame.can_id = 0x004; // ID 4
    frame.can_dlc = 8;
    frame.data[0] = msg.data;
    for (int i = 1; i < 8; i++) { // 8 Byte
        frame.data[i] = 0;
    }

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        state = msg.data;
        return;
    }

    if (close(s) < 0) {
        perror("Close");
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "orthos_halo_control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::Subscriber state_command_sub = n.subscribe("state_command", 1,
                                                    change_state);

    ros::Publisher state_publisher = n.advertise<std_msgs::Int16>("state", 1); // Publishes the currently set state
    ros::Publisher state_text_publisher = n.advertise<std_msgs::String>("state_string", 1); // Publishes it again as decrypted text


    while (ros::ok()) {
        std_msgs::Int16 current_state;
        std_msgs::String current_state_string;
        current_state.data = state;
        current_state_string.data = states[state];
        state_publisher.publish(current_state);
        state_text_publisher.publish(current_state_string);

        loop_rate.sleep();
        ros::spinOnce();
    }

}

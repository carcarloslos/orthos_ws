#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/ColorRGBA.h>
#include <cstdio>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>

int state = 0; // Global state variable, will be changed each time a value is send to the OCU, off by default
int rgba[5] {0, 255, 0, 0, 255}; // Global variable, will be changed each time a new value is sent via a topic [r, g, b, min, max]
// State:	0 = OFF	1 = Const. On	2 = Rotate	3 = PingPong	4 = Fade	5 = Gay	6 = Party
const char* states[8] = {"off", "cnston", "rotate", "pingpong", "fade", "gay", "party"};

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

    frame.can_id = 0x005; // ID 5 for chassis lighting
    frame.can_dlc = 8;
    state = msg.data;
    frame.data[0] = state;
    for (int i = 0; i < 5; i++) { // 5 byte
        frame.data[i] = rgba[i];
    }
    for (int i = 6; i < 8; i++) {
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


void change_rgba(const std_msgs::ColorRGBA &msg) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;


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

    frame.can_id = 0x005; // ID 5 for chassis lighting
    frame.can_dlc = 8;
    frame.data[0] = state;
    rgba[0] = msg.r;
    rgba[1] = msg.g;
    rgba[2] = msg.b;
    rgba[3] = 0;
    rgba[4] = 255;
    for (int i = 0; i < 5; i++) { // 5 byte
        frame.data[i] = rgba[i];
    }
    for (int i = 6; i < 8; i++) {
    	frame.data[i] = 0;
    }


    if (close(s) < 0) {
        perror("Close");
        return;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "orthos_chassis_lighting_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::Subscriber state_command_sub = n.subscribe("chassis_state_command", 1,
                                                    change_state);
    ros::Subscriber rgba_command_sub = n.subscribe("chassis_rgba_command", 1, change_rgba);

    ros::Publisher state_publisher = n.advertise<std_msgs::Int16>("chassis_state", 1); // Publishes the currently set state
    ros::Publisher state_text_publisher = n.advertise<std_msgs::String>("chassis_state_string", 1); // Publishes it again as decrypted text
    ros::Publisher rgba_publisher = n.advertise<std_msgs::ColorRGBA>("chassis_rgba", 1); // Publishes the currently set rgba command

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

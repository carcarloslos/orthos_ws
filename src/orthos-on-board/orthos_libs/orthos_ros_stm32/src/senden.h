#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <math.h>
//#define M_PI 3.14159265358979323846

float deg2rad(float grad)
{
    return grad*(M_PI/180);
}

float rad2deg(float rad)
{
    return rad*(180/M_PI);
}

int senden(int motor, float angle)
{
	uint8_t motor_u8 = motor;
	//if(angle<M_PI)
	//{
	//	angle = -angle;
	//}
	float angle_100 = rad2deg(angle)*(100/45);
	uint8_t *angle_u8;
	angle_u8 = reinterpret_cast<uint8_t*>(&angle_100);
                   
	int s; 
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	printf("Motor %d || rad %f, deg %f --> %f \n", motor, angle, rad2deg(angle), angle_100);  

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "can1" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

	frame.can_id = 0x003;
	frame.can_dlc = 8;
	frame.data[0] = motor;
	for (int i=1; i<5; i++)
	{
		frame.data[i] = angle_u8[i-1];
	}
	for (int i=5; i<8; i++)
	{
		frame.data[i] = 0;
	}
	

	if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write");
		return 1;
	}

	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

	return 0;
}

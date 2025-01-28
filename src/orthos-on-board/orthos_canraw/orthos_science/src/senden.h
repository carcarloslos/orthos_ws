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

int senden(int channel, float data_value)
{
	uint8_t channel_u8 = channel;
	uint8_t *data_value_u8;
	data_value_u8 = reinterpret_cast<uint8_t*>(&data_value);
                   
	int s; 
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

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

	frame.can_id = 0x006; // 0x006 = science payload
	frame.can_dlc = 8;    // byte 0 is the channel, rest are data
	frame.data[0] = channel;
	for (int i=1; i<5; i++)
	{
		frame.data[i] = data_value_u8[i-1];
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

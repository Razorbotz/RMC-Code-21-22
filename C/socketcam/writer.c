#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int
main(void)
{
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	const char *ifname = "can0";

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	printf("Bound to socket \n");

	frame.can_id  = 0x123;
	frame.can_dlc = 4;
	frame.data[0] = 0x08;
	frame.data[1] = 0x04;
	frame.data[2] = 0x14;
	frame.data[3] = 0x80;

	for(int address=0;address<4096;address++){
		frame.can_id  = address;
    		nbytes = write(s, &frame, sizeof(struct can_frame));
		if(nbytes==-1){
			address--;
			continue;	
 		}

		printf("%X:Wrote %d bytes\n", address,nbytes);
	}
	
	return 0;
}

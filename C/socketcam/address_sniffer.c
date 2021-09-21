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

#include <ncurses.h>
#include <vector>

std::vector<uint32_t> addresses;

void printFrame(struct can_frame frame){
	printw("%X:  ",frame.can_id);
	for(int index=0;index<frame.can_dlc;index++){
		printw("%02X ",frame.data[index]);
	}
}

bool onList(uint32_t address){
	for(uint32_t storedAddress:addresses){
		if(storedAddress==address){
			return true;
		}
	}
	
	return false;
}


int main(void) {
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	initscr();
	curs_set(0);

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
	int row=0;
	int col=0;
	while(1){
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if(nbytes==-1) continue; 

		if(onList(frame.can_id)){
			continue;	
		}
		addresses.push_back(frame.can_id);

		move(row++,col);

		if(row==15){
			row=0;
			col+=10;
		}
		printw("%X", frame.can_id);
		refresh();
	}

	return 0;
}

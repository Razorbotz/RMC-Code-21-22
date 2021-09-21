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

void printFrame(struct can_frame frame){
	printw("%X:  ",frame.can_id);
	for(int index=0;index<frame.can_dlc;index++){
		printw("%02X ",frame.data[index]);
	}
}

void parseFrame(struct can_frame frame){
	float currentScalar = 0.125f;
	int i1=frame.data[0];
	i1=i1<<2;
	i1=i1|(frame.data[1]>>6 & 0x03);
	float current1=i1*currentScalar;	
	printw(" %2.3f",current1);
	
	int i2=frame.data[1] & 0x3f;
	i2=i2<<4;
	i2=i2|(frame.data[2]>>4 & 0x0f);
	float current2=i2*currentScalar;	
	printw(" %2.3f",current2);

	int i3=frame.data[2] & 0x0f;
	i3=i3<<6;
	i3=i3|(frame.data[3]>>2 & 0x3f);
	float current3=i3*currentScalar;	
	printw(" %2.3f",current3);

	int i4=frame.data[3] & 0x03;
	i4=i4<<8;
	i4=i4|(frame.data[4]);
	float current4=i4*currentScalar;	
	printw(" %2.3f",current4);

	int i5=frame.data[5];
	i5=i5<<2;
	i5=i5|(frame.data[6]>>6 & 0x03);
	float current5=i5*currentScalar;	
	printw(" %2.3f",current5);

	int i6=frame.data[6] & 0x3f;
	i6=i6<<4;
	i6=i6|(frame.data[7]>>4 & 0x7);
	float current6=i6*currentScalar;	
	printw(" %2.3f",current6);
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

	while(1){
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if(nbytes==-1) continue; 

		move(0,0);
		printw("bytes read: %d", nbytes);
		printw("   address: %X", frame.can_id);
		printw("   payload byte count: %X", frame.can_dlc);

		if(frame.can_id==0x88041401){
			move(3,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041441){
			move(4,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041481){
			move(5,3);
			printFrame(frame);
			printw("   %2.2fV",.05*frame.data[6]+4);
		}
		if(frame.can_id==0x880414C1){
			move(6,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041501){
			move(7,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041541){
			move(8,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041581){
			move(9,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x880415C1){
			move(10,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041601){
			move(11,3);
			printFrame(frame);
			parseFrame(frame);
		}
		if(frame.can_id==0x88041641){
			move(12,3);
			printFrame(frame);
			printw("      ");
			parseFrame(frame);
		}

		refresh();
        }
	
	return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float64.h>

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
#include <chrono>

#include "PowerDistributionPanel.hpp"
#include "messages/Power.h"


int main(int argc, char **argv){

	ros::init(argc,argv,"power_distribution_panel_node");
	ros::NodeHandle nodeHandle;
	ros::Publisher publisher=nodeHandle.advertise<messages::Power>("power",1);
	messages::Power power;

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


        PowerDistributionPanel pdp;

	auto start = std::chrono::high_resolution_clock::now();
	while(ros::ok()){
		nbytes = read(s, &frame, sizeof(struct can_frame));
                if(nbytes==-1) continue;

                pdp.parseFrame(frame);
//std::cout << pdp.getVoltage() << "   "  << pdp.getCurrentC(0)<< std::endl;
		auto finish = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
			power.voltage=pdp.getVoltage();
			power.current0=pdp.getCurrentC(0);
			power.current1=pdp.getCurrentC(1);
			power.current2=pdp.getCurrentC(2);
			power.current3=pdp.getCurrentC(3);
			power.current4=pdp.getCurrentC(4);
			power.current5=pdp.getCurrentC(5);
			power.current6=pdp.getCurrentC(6);
			power.current7=pdp.getCurrentC(7);
			power.current8=pdp.getCurrentC(8);
			power.current9=pdp.getCurrentC(9);
			power.current10=pdp.getCurrentC(10);
			power.current11=pdp.getCurrentC(11);
			power.current12=pdp.getCurrentC(12);
			power.current13=pdp.getCurrentC(13);
			power.current14=pdp.getCurrentC(14);
			power.current15=pdp.getCurrentC(15);
//std::cout << "sending " << power.voltage << std::endl;
			publisher.publish(power);
			start = std::chrono::high_resolution_clock::now();
		}
		ros::spinOnce();
	}
}

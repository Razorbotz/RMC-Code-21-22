#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

int fd;
bool GO=false;
bool forward = true;
bool shoulderL = false;
bool shoulderR = false;
float shoulderSpeed = 0.0;
uint8_t drum_speed_old = 0;
uint8_t drumVar=0;

void stopCallback(std_msgs::Empty empty){
        std::cout << "STOP" << std::endl;
	write(fd,"E",1);
	drumVar = 0;
        GO=false;
}
void goCallback(std_msgs::Empty empty){
        std::cout << "GO" << std::endl;
        write(fd,"G",1);
        GO=true;

}
void drumSpeedCallback(const std_msgs::UInt8& drum_speed){
	std::cout << "drum_speed " << drum_speed << std::endl;
	if(drum_speed.data > drum_speed_old)
	    forward =true;
	else
	    forward =false;

	if(forward && drum_speed.data >= drumVar+5){
	    drumVar = drumVar+5;
	    
	    write(fd,"+",1);
            }
        else if(!forward && drum_speed.data <= drumVar-5){
            drumVar = drumVar-5;
            write(fd,"-",1);
        }
	drum_speed_old = drum_speed.data;

/*

            std::cout << "speed " << drum_speed << std::endl;
	    if(drumVar < drum_speed.data){
	        drumVar = drumVar+5;
	        write(fd,"+",1);
	    }
	    else if(drumVar > drum_speed.data){
	        drumVar = drumVar-5;
	        write(fd,"-",1);
	    }
	}// */
}

void scoopDirCallback(const std_msgs::Int8 scoopDir){
        std::cout << "scoop_dir " << scoopDir;
	switch(scoopDir.data){
	    case -1:
	        std::cout << " backward" << scoopDir << std::endl;
	        write(fd,"b",1);
		break;
	    case 0:
                std::cout << " stop" << scoopDir << std::endl;
		write(fd,"s",1);
		break;
	    case 1:
                std::cout << " forward" << std::endl;
		write(fd,"f",1);
		break;
	}

}

void updateShoulder(){
	float deadZone = 0.2;
        if (!shoulderL && !shoulderR){
//	    std::cout << "Update " << shoulderSpeed << std::endl;
	    if (shoulderSpeed > deadZone)
		write(fd,"U",1);
	    else if(shoulderSpeed < -deadZone)
		write(fd,"D",1);
	    else
		write(fd,"S",1);
	}
	else if (shoulderL){
	    if (shoulderSpeed > deadZone)
		write(fd,"l",1);
	    else if(shoulderSpeed < -deadZone)
		write(fd,"L",1);
	    else
                write(fd,"S",1);
	}
	else if (shoulderR){
            if (shoulderSpeed > deadZone)
                write(fd,"r",1);
            else if(shoulderSpeed < -deadZone)
                write(fd,"R",1);
	    else
                write(fd,"S",1);
        }

}

void shoulderCallback(const std_msgs::Float32 shoulder){
	std::cout << "shoulder " << shoulder << std::endl;
	shoulderSpeed = shoulder.data;
	updateShoulder();
}

void shoulderLeftGoCallback(const std_msgs::Bool shoulderLGo){
	std::cout << "shoulder_L " << shoulderLGo.data << std::endl;
	shoulderL = shoulderLGo.data;
	updateShoulder();
}

void shoulderRightGoCallback(const std_msgs::Bool shoulderRGo){
	std::cout << "shoulder_R " << shoulderRGo.data << std::endl;
	shoulderR = shoulderRGo.data;
	updateShoulder();
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls 
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8-bit characters 
    tty.c_cflag &= ~PARENB;     // no parity bit 
    tty.c_cflag &= ~CSTOPB;     // only need 1 stop bit 
    tty.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol 

    // setup for non-canonical mode 
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    // fetch bytes as they become available 
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

//=================================================================================================
int main(int argc,char** argv){
        ros::init(argc,argv,"scoop_drum");
	ros::NodeHandle nodeHandle;
	ros::Subscriber stopSubscriber=nodeHandle.subscribe("STOP",1,stopCallback);
        ros::Subscriber goSubscriber=nodeHandle.subscribe("GO",1,goCallback);
	ros::Subscriber drumSpeedSubscriber=nodeHandle.subscribe("drum_speed",1,drumSpeedCallback);
        ros::Subscriber scoopDirSubscriber=nodeHandle.subscribe("scoop_dir",1,scoopDirCallback);
	ros::Subscriber shoulderSpeedSubscriber=nodeHandle.subscribe("shoulder_speed",1,shoulderCallback);
	ros::Subscriber shoulderLeftGoSubscriber=nodeHandle.subscribe("shoulder_L_Go",1,shoulderLeftGoCallback);
	ros::Subscriber shoulderRightGoSubscriber=nodeHandle.subscribe("shoulder_R_Go",1,shoulderRightGoCallback);	

	char *portname = "/dev/ttyS0";
//	int wlen;

	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
	  printf("Error opening %s: %s\n", portname, strerror(errno));
	  return -1;
	}

        set_interface_attribs(fd, B115200);
		
	write(fd,"E",1);// */
        std::cout << "START: " << std::endl;
//	std::cout << "portname: " << portname << std::endl;

        ros::Rate rate(10);
	while(ros::ok()){
                ros::spinOnce();
        }
}

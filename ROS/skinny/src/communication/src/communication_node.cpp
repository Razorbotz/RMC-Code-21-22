#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <linux/reboot.h>
#include <sys/reboot.h>




#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <messages/Power.h>
#include <messages/KeyState.h>
#include <messages/HatState.h>
#include <messages/ButtonState.h>
#include <messages/AxisState.h>
#include <messages/TalonOut.h>
#include <messages/VictorOut.h>


#define PORT 31337

std_msgs::Empty empty;
bool silentRunning=true;

void insert(float value,uint8_t* array){
        array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
        array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
        array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
        array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}
void insert(int value,uint8_t* array){
        array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
        array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
        array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
        array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}

float parseFloat(uint8_t* array){
 	uint32_t axisYInteger=0;
	axisYInteger|=uint32_t(array[0])<<24;	
	axisYInteger|=uint32_t(array[1])<<16;	
	axisYInteger|=uint32_t(array[2])<<8;	
	axisYInteger|=uint32_t(array[3])<<0;	
	float value=(float)*(static_cast<float*>(static_cast<void*>(&axisYInteger)));

	return value;
}


int parseInt(uint8_t* array){
 	uint32_t axisYInteger=0;
	axisYInteger|=uint32_t(array[0])<<24;	
	axisYInteger|=uint32_t(array[1])<<16;	
	axisYInteger|=uint32_t(array[2])<<8;	
	axisYInteger|=uint32_t(array[3])<<0;	
	int value=(int)*(static_cast<int*>(static_cast<void*>(&axisYInteger)));

	return value;
}

 
int new_socket;
void powerCallback(const messages::Power power){
//	std::cout << "power " << power.voltage << std::endl;
	if(silentRunning)return;
	int messageSize=17*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=1;
	float* floatPointer=(float*)(void*)&message[2];

	insert(power.voltage,(uint8_t*)floatPointer++);
	insert(power.current0,(uint8_t*)floatPointer++);
	insert(power.current1,(uint8_t*)floatPointer++);
	insert(power.current2,(uint8_t*)floatPointer++);
	insert(power.current3,(uint8_t*)floatPointer++);
	insert(power.current4,(uint8_t*)floatPointer++);
	insert(power.current5,(uint8_t*)floatPointer++);
	insert(power.current6,(uint8_t*)floatPointer++);
	insert(power.current7,(uint8_t*)floatPointer++);
	insert(power.current8,(uint8_t*)floatPointer++);
	insert(power.current9,(uint8_t*)floatPointer++);
	insert(power.current10,(uint8_t*)floatPointer++);
	insert(power.current11,(uint8_t*)floatPointer++);
	insert(power.current12,(uint8_t*)floatPointer++);
	insert(power.current13,(uint8_t*)floatPointer++);
	insert(power.current14,(uint8_t*)floatPointer++);
	insert(power.current15,(uint8_t*)floatPointer++);
	
	send(new_socket, message, messageSize, 0);
}


void talon1Callback(const messages::TalonOut talonOut){
//std::cout << "talon1Callback" << std::endl;	
	if(silentRunning)return;
	int messageSize=11*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=2;
	float* byte4Pointer=(float*)(void*)&message[2];

	insert(talonOut.deviceID,(uint8_t*)byte4Pointer++);
	insert(talonOut.busVoltage,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputCurrent,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputVoltage,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputPercent,(uint8_t*)byte4Pointer++);
	insert(talonOut.temperature,(uint8_t*)byte4Pointer++);
	insert(talonOut.sensorPosition,(uint8_t*)byte4Pointer++);
	insert(talonOut.sensorVelocity,(uint8_t*)byte4Pointer++);
	insert(talonOut.closedLoopError,(uint8_t*)byte4Pointer++);
	insert(talonOut.integralAccumulator,(uint8_t*)byte4Pointer++);
	insert(talonOut.errorDerivative,(uint8_t*)byte4Pointer++);

	send(new_socket, message, messageSize, 0);
}


void talon2Callback(const messages::TalonOut talonOut){
	if(silentRunning)return;
	int messageSize=11*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=3;
	float* byte4Pointer=(float*)(void*)&message[2];

	insert(talonOut.deviceID,(uint8_t*)byte4Pointer++);
	insert(talonOut.busVoltage,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputCurrent,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputVoltage,(uint8_t*)byte4Pointer++);
	insert(talonOut.outputPercent,(uint8_t*)byte4Pointer++);
	insert(talonOut.temperature,(uint8_t*)byte4Pointer++);
	insert(talonOut.sensorPosition,(uint8_t*)byte4Pointer++);
	insert(talonOut.sensorVelocity,(uint8_t*)byte4Pointer++);
	insert(talonOut.closedLoopError,(uint8_t*)byte4Pointer++);
	insert(talonOut.integralAccumulator,(uint8_t*)byte4Pointer++);
	insert(talonOut.errorDerivative,(uint8_t*)byte4Pointer++);

	send(new_socket, message, messageSize, 0);
}


void victor1Callback(const messages::VictorOut victorOut){
	if(silentRunning)return;
	int messageSize=11*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=4;
	float* byte4Pointer=(float*)(void*)&message[2];

	insert(victorOut.deviceID,(uint8_t*)byte4Pointer++);
	insert(victorOut.busVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputPercent,(uint8_t*)byte4Pointer++);

	send(new_socket, message, messageSize, 0);
}


void victor2Callback(const messages::VictorOut victorOut){
	if(silentRunning)return;
	int messageSize=11*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=5;
	float* byte4Pointer=(float*)(void*)&message[2];

	insert(victorOut.deviceID,(uint8_t*)byte4Pointer++);
	insert(victorOut.busVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputPercent,(uint8_t*)byte4Pointer++);

	send(new_socket, message, messageSize, 0);
}


void victor3Callback(const messages::VictorOut victorOut){
	if(silentRunning)return;
	int messageSize=11*4+2;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=6;
	float* byte4Pointer=(float*)(void*)&message[2];

	insert(victorOut.deviceID,(uint8_t*)byte4Pointer++);
	insert(victorOut.busVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputVoltage,(uint8_t*)byte4Pointer++);
	insert(victorOut.outputPercent,(uint8_t*)byte4Pointer++);

	send(new_socket, message, messageSize, 0);
}


uint8_t driveState=1;
void driveStateCallback(const std_msgs::Bool state){
	int messageSize=3;
	uint8_t message[messageSize];
	message[0]=messageSize;
	message[1]=7;
	message[2]=(uint8_t)state.data;
	driveState=(uint8_t)state.data;
	
	send(new_socket, message, messageSize, 0);
}


void sendRobotState() {
    int messageSize = 3;
    uint8_t message[messageSize];
    message[0] = messageSize;
    message[1] = 7;
    message[2] = driveState;

    send(new_socket, message, messageSize, 0);
}


std::string getAddressString(int family, std::string interfaceName){
    std::string addressString("");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        if(strcmp(interfaceAddresses->ifa_name,interfaceName.c_str())==0 && interfaceAddresses->ifa_addr->sa_family == family) {
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET) {
                sockaddr_in *socketAddress = reinterpret_cast<sockaddr_in *>(interfaceAddresses->ifa_addr);
                addressString += inet_ntoa(socketAddress->sin_addr);
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_INET6) {
                sockaddr_in6 *socketAddress = reinterpret_cast<sockaddr_in6 *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < 16; index += 2) {
                    char bits[5];
                    sprintf(bits,"%02x%02x", socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index + 1]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
            if (interfaceAddresses->ifa_addr->sa_family == AF_PACKET) {
                sockaddr_ll *socketAddress = reinterpret_cast<sockaddr_ll *>(interfaceAddresses->ifa_addr);
                for (int index = 0; index < socketAddress->sll_halen; index++) {
                    char bits[3];
                    sprintf(bits,"%02x", socketAddress->sll_addr[index]);
                    if (index)addressString +=":";
                    addressString +=bits;
                }
            }
        }
    }
    freeifaddrs(interfaceAddresses);
    return addressString;
}


void printAddresses() {
    printf("Addresses\n");
    ifaddrs* interfaceAddresses = nullptr;
    for (int failed=getifaddrs(&interfaceAddresses); !failed && interfaceAddresses; interfaceAddresses=interfaceAddresses->ifa_next){
        printf("%s ",interfaceAddresses->ifa_name);
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET){
            printf("AF_INET ");
            sockaddr_in* socketAddress=reinterpret_cast<sockaddr_in*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin_port);
            printf("%s ",inet_ntoa(socketAddress->sin_addr));
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_INET6){
            printf("AF_INET6 ");
            sockaddr_in6* socketAddress=reinterpret_cast<sockaddr_in6*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sin6_port);
            printf("%d ",socketAddress->sin6_flowinfo);
            for(int index=0;index<16;index+=2) {
                if(index)printf(":");
                printf("%02x%02x",socketAddress->sin6_addr.s6_addr[index],socketAddress->sin6_addr.s6_addr[index+1]);
            }
        }
        if(interfaceAddresses->ifa_addr->sa_family == AF_PACKET){
            printf("AF_PACKET ");
            sockaddr_ll* socketAddress=reinterpret_cast<sockaddr_ll*>(interfaceAddresses->ifa_addr);
            printf("%d ",socketAddress->sll_protocol);
            printf("%d ",socketAddress->sll_ifindex);
            printf("%d ",socketAddress->sll_hatype);
            printf("%d ",socketAddress->sll_pkttype);
            for(int index=0;index<socketAddress->sll_halen;index++){
                if(index)printf(":");
                printf("%02x",socketAddress->sll_addr[index]);
            }
        }
        printf("\n");
    }
    printf("Done\n");
}

void reboot(){
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
}

std::string robotName="Skinny";
bool broadcast=true;
void broadcastIP(){
    while(true){
        if(broadcast){
            std::string addressString=getAddressString(AF_INET,"wlan0");

            std::string message(robotName+"@"+addressString);
            std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

            if(socket>=0){
                struct sockaddr_in socketAddress;
                socketAddress.sin_family=AF_INET;
                socketAddress.sin_addr.s_addr = inet_addr("226.1.1.1");
                socketAddress.sin_port = htons(4321);

                struct in_addr localInterface;
                localInterface.s_addr = inet_addr(addressString.c_str());
                if(setsockopt(socketDescriptor, IPPROTO_IP, IP_MULTICAST_IF, (char*)&localInterface, sizeof(localInterface))>=0){
                    sendto(socketDescriptor,message.c_str(),message.length(),0,(struct sockaddr*)&socketAddress, sizeof(socketAddress));
                }
            }
            close(socketDescriptor);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}


int main(int argc, char **argv){
	std::cout << "Started communication" << std::endl;
	ros::init(argc,argv,"communication");

	ros::NodeHandle nodeHandleParameters("~");

	bool success;
	//success=nodeHandleParameters.getParam("robot_name", robotName);
	std::cout << success << " robotName: " << robotName << std::endl;


	ros::NodeHandle nodeHandle;
    success=nodeHandle.getParam("/communication/robot_name",robotName);
    std::cout << success << " robotName: " << robotName << std::endl;

//	ros::Publisher joystick1RollPublisher=nodeHandle.advertise<std_msgs::Float32>("joystick_1_roll",1);
//	ros::Publisher joystick1PitchPublisher=nodeHandle.advertise<std_msgs::Float32>("joystick_1_pitch",1);
//	ros::Publisher joystick1YawPublisher=nodeHandle.advertise<std_msgs::Float32>("joystick_1_yaw",1);
//	ros::Publisher joystick1ThrottlePublisher=nodeHandle.advertise<std_msgs::Float32>("joystick_1_throttle",1);
//	ros::Publisher joystick1HatPublisher=nodeHandle.advertise<messages::HatState>("joystick_1_hat",1);
//	ros::Publisher joystick1ButtonPublisher=nodeHandle.advertise<messages::ButtonState>("joystick_1_button",1);

    ros::Publisher joystickAxisPublisher=nodeHandle.advertise<messages::AxisState>("joystick_axis",1);
    ros::Publisher joystickHatPublisher=nodeHandle.advertise<messages::HatState>("joystick_hat",1);
    ros::Publisher joystickButtonPublisher=nodeHandle.advertise<messages::ButtonState>("joystick_button",1);

    ros::Publisher keyPublisher=nodeHandle.advertise<messages::KeyState>("key",1);

    ros::Publisher stopPublisher=nodeHandle.advertise<std_msgs::Empty>("STOP",1);
	ros::Publisher goPublisher=nodeHandle.advertise<std_msgs::Empty>("GO",1);

	ros::Subscriber powerSubscriber=nodeHandle.subscribe("power",1,powerCallback);
	ros::Subscriber talon1Subscriber=nodeHandle.subscribe("talon_1_info",1,talon1Callback);
	ros::Subscriber talon2Subscriber=nodeHandle.subscribe("talon_2_info",1,talon2Callback);
	ros::Subscriber victor1Subscriber=nodeHandle.subscribe("victor_3_info",1,victor1Callback);
	ros::Subscriber victor2Subscriber=nodeHandle.subscribe("victor_4_info",1,victor2Callback);
	ros::Subscriber victor3Subscriber=nodeHandle.subscribe("victor_6_info",1,victor3Callback);
	ros::Subscriber driveStateSubscriber=nodeHandle.subscribe("drive_state",1,driveStateCallback);

	int server_fd, bytesRead; 
	struct sockaddr_in address; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	uint8_t buffer[1024] = {0}; 
	std::string hello("Hello from server");

	// Creating socket file descriptor
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { 
        	perror("socket failed"); 
        	exit(EXIT_FAILURE); 
	}
    std::thread broadcastThread(broadcastIP);

	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons( PORT ); 

	if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) { 
        	perror("bind failed"); 
        	exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) { 
        	perror("listen"); 
       		exit(EXIT_FAILURE); 
	} 
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
        	perror("accept"); 
        	exit(EXIT_FAILURE); 
	}
	broadcast=false;
	bytesRead = read(new_socket, buffer, 1024); 
	printf("%s\n",buffer ); 
	send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
	printf("Hello message sent\n"); 
	silentRunning=true;
	sendRobotState();

	fcntl(new_socket, F_SETFL, O_NONBLOCK);

	//ros::Rate rate(10);
	//double speed;
	//double increment=.01;
	std::list<uint8_t> messageBytesList;
	uint8_t message[256];
	while(ros::ok()){
		bytesRead = recv(new_socket, buffer, 1024, 0);
		for(int index=0;index<bytesRead;index++){
			messageBytesList.push_back(buffer[index]);
		}

//		if(bytesRead>0){
//			std::cout << std::endl;
//			std::cout << "size " << messageBytesList.size() << std::endl;
//			std::cout << "front " << (int)messageBytesList.front() << std::endl;
//		}
		
		if(bytesRead==0){
			stopPublisher.publish(empty);
			std::cout << "Lost connection " << std::endl;
			broadcast=true;
			//wait for reconnect
			if (listen(server_fd, 3) < 0) { 
        			perror("listen"); 
       				exit(EXIT_FAILURE); 
			} 
			if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) { 
        			perror("accept"); 
        			exit(EXIT_FAILURE); 
			}
			broadcast=false;
			bytesRead = read(new_socket, buffer, 1024); 
			std::cout << buffer << std::endl;
			send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
			std::cout << "Hello message sent" << std::endl;
			fcntl(new_socket, F_SETFL, O_NONBLOCK);
	
			silentRunning=true;
			sendRobotState();
		}

		while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
			//std::cout << "bytes read " << bytesRead << std::endl;
		 	int messageSize=messageBytesList.front();	
			messageBytesList.pop_front();
			messageSize--;
			for(int index=0;index<messageSize;index++){
				message[index]=messageBytesList.front();
				messageBytesList.pop_front();
			}
			//parse command
			uint8_t command=message[0];
			if(command==1){
			    messages::AxisState axisState;
			    axisState.joystick=message[1];
			    axisState.axis=message[2];
			    axisState.state=parseFloat(&message[3]);

                joystickAxisPublisher.publish(axisState);
                //std::cout << buffer[0] << " axis " << axisState.joystick << " " << axisState.axis << " " << axisState.state  << std::endl;
			}

            if(command==2){
                messages::KeyState keyState;
                keyState.key=((uint16_t)message[1])<<8 | ((uint16_t)message[2]);
                keyState.state=message[3];
                keyPublisher.publish(keyState);
                //std::cout << buffer[0] << " key " << keyState.key << " " << keyState.state  << std::endl;
            }
//			if(command==1){
//				std_msgs::Float32 roll;
//				roll.data=parseFloat(&message[1]);
//				joystick1RollPublisher.publish(roll);
////				std::cout << buffer[0] << "roll  " << roll.data << std::endl;
//			}
//			if(command==2){
//				std_msgs::Float32 pitch;
//				pitch.data=parseFloat(&message[1]);
//				joystick1PitchPublisher.publish(pitch);
////				std::cout << buffer[0] << "pitch  " << pitch.data << std::endl;
//			}
//			if(command==3){
//				std_msgs::Float32 throttle;
//				throttle.data=parseFloat(&message[1]);
//				joystick1ThrottlePublisher.publish(throttle);
////				std::cout << buffer[0] << "throttle  " << throttle.data << std::endl;
//			}
//			if(command==4){
//				std_msgs::Float32 yaw;
//				yaw.data=parseFloat(&message[1]);
//				joystick1YawPublisher.publish(yaw);
////				std::cout << buffer[0] << "yaw  " << yaw.data << std::endl;
//			}


			if(command==5){
				messages::ButtonState buttonState;
				buttonState.joystick=message[1];
				buttonState.button=message[2];
				buttonState.state=message[3];
				if(buttonState.button==0 && buttonState.state==0){
					std::cout << "publish stop" << std::endl;
					stopPublisher.publish(empty);
				}	
				if(buttonState.button==0 && buttonState.state==1){
					std::cout << "publish go" << std::endl;
					goPublisher.publish(empty);
				}	
				//joystick1ButtonPublisher.publish(buttonState);
                joystickButtonPublisher.publish(buttonState);
				std::cout << buffer[0] << " button  " << buttonState.joystick << " " << buttonState.button << " " << buttonState.state  << std::endl;
			}
			if(command==6){
				messages::HatState hatState;
				hatState.joystick=message[1];
				hatState.hat=message[2];
				hatState.state=message[3];
				//joystick1HatPublisher.publish(hatState);
                joystickHatPublisher.publish(hatState);
//				std::cout << buffer[0] << " hat  " << hatState.state << std::endl;
                std::cout << buffer[0] << " hat  " << hatState.joystick << " " << hatState.hat << " " << hatState.state  << std::endl;
            }
			if(command==7){
				silentRunning=message[1];
				std::cout << "silentRunning " << silentRunning << std::endl;
			}
            if(command==8){
                reboot();
                std::cout << "reboot " << silentRunning << std::endl;
            }
		}

		ros::spinOnce();
	}

    broadcastThread.join();
}

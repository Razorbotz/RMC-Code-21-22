#include <unistd.h>
#include <errno.h>
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
#include <vector>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <messages/msg/power.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/hat_state.hpp>  
#include <messages/msg/button_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/talon_out.hpp>
#include <messages/msg/victor_out.hpp>
#include <messages/msg/zed_position.hpp>

#include <BinaryMessage.hpp>

#define PORT 31337

/** @file
 * @brief Brief description of file
 * Detailed description of file 
 * 
 * */

std_msgs::msg::Empty empty;
bool silentRunning=true;
int new_socket;
rclcpp::Node::SharedPtr nodeHandle;

/** @brief Brief description of function
 * Detailed description of function
 * @param array
 * @return void
 * */
void insert(float value,uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}

/** @brief Brief description of function
 * Detailed description of function
 * @param array
 * @return void
 * */
void insert(int value, uint8_t* array){
    array[0]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>24) & 0xff);
    array[1]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>16) & 0xff);
    array[2]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>8) & 0xff);
    array[3]=uint8_t((uint32_t(*(static_cast<uint32_t*>(static_cast<void*>(&value))))>>0) & 0xff);
}

/** @brief Brief description of function
 * Detailed description of function
 * @param array
 * @return value
 * */
float parseFloat(uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    float value=(float)*(static_cast<float*>(static_cast<void*>(&axisYInteger)));

    return value;
}

/** @brief Brief description of function
 * Detailed description of function
 * @param array
 * @return value
 * */
int parseInt(uint8_t* array){
    uint32_t axisYInteger=0;
    axisYInteger|=uint32_t(array[0])<<24;    
    axisYInteger|=uint32_t(array[1])<<16;    
    axisYInteger|=uint32_t(array[2])<<8;    
    axisYInteger|=uint32_t(array[3])<<0;    
    int value=(int)*(static_cast<int*>(static_cast<void*>(&axisYInteger)));

    return value;
}

 

void send(BinaryMessage message){
    //RCLCPP_INFO(nodeHandle->get_logger(), "send message");
    std::shared_ptr<std::list<uint8_t>> byteList = message.getBytes();

    std::vector<uint8_t> bytes(byteList->size());
    int index = 0;
    for(auto byteIterator = byteList->begin(); byteIterator != byteList->end(); byteIterator++, index++){
        bytes.at(index) = *byteIterator;
    }

    //RCLCPP_INFO(nodeHandle->get_logger(), "sending %s   bytes = %d", message.getLabel().c_str(), byteList->size());
    if(send(new_socket, bytes.data(), byteList->size(), 0)== -1){
        std::cout << "failed to send message " << errno << std::endl;	    
    }
}


void send(std::string messageLabel, const messages::msg::VictorOut::SharedPtr victorOut){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send victor");
    BinaryMessage message(messageLabel);

    message.addElementFloat32("Device ID",victorOut->device_id);
    message.addElementFloat32("Bus Voltage",victorOut->bus_voltage);
    message.addElementFloat32("Output Voltage",victorOut->output_voltage);
    message.addElementFloat32("Output Percent",victorOut->output_percent);

    send(message);
}


void send(std::string messageLabel, const messages::msg::TalonOut::SharedPtr talonOut){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send talon");
    BinaryMessage message(messageLabel);

    message.addElementFloat32("Device ID",talonOut->device_id);
    message.addElementFloat32("Bus Voltage",talonOut->bus_voltage);
    message.addElementFloat32("Output Current",talonOut->output_current);
    message.addElementFloat32("Output Voltage",talonOut->output_voltage);
    message.addElementFloat32("Output Percent",talonOut->output_percent);
    message.addElementFloat32("Temperature",talonOut->temperature);
    message.addElementFloat32("Sensor Position",talonOut->sensor_position);
    message.addElementFloat32("Sensor Velocity",talonOut->sensor_velocity);
    message.addElementFloat32("closed Loop Error",talonOut->closed_loop_error);
    message.addElementFloat32("Integral Accumulator",talonOut->integral_accumulator);
    message.addElementFloat32("Error Derivative",talonOut->error_derivative);

    send(message);
}


void send(std::string messageLabel, const messages::msg::Power::SharedPtr power){
    if(silentRunning)return;
    //RCLCPP_INFO(nodeHandle->get_logger(), "send power");
    BinaryMessage message(messageLabel);

    message.addElementFloat32("Voltage",power->voltage);
    message.addElementFloat32("Current 0",power->current0);
    message.addElementFloat32("Current 1",power->current1);
    message.addElementFloat32("Current 2",power->current2);
    message.addElementFloat32("Current 3",power->current3);
    message.addElementFloat32("Current 4",power->current4);
    message.addElementFloat32("Current 5",power->current5);
    message.addElementFloat32("Current 6",power->current6);
    message.addElementFloat32("Current 7",power->current7);
    message.addElementFloat32("Current 8",power->current8);
    message.addElementFloat32("Current 9",power->current9);
    message.addElementFloat32("Current 10",power->current10);
    message.addElementFloat32("Current 11",power->current11);
    message.addElementFloat32("Current 12",power->current12);
    message.addElementFloat32("Current 13",power->current13);
    message.addElementFloat32("Current 14",power->current14);
    message.addElementFloat32("Current 15",power->current15);

    send(message);
}


void zedPositionCallback(const messages::msg::ZedPosition::SharedPtr zedPosition){
    if(silentRunning)return;
    BinaryMessage message("Zed");
    message.addElementFloat32("X", zedPosition->x);
    message.addElementFloat32("Y", zedPosition->y);
    message.addElementFloat32("Z", zedPosition->z);
    message.addElementFloat32("oX", zedPosition->ox);
    message.addElementFloat32("oY", zedPosition->oy);
    message.addElementFloat32("oZ", zedPosition->oz);
    message.addElementFloat32("oW", zedPosition->ow);
    message.addElementBoolean("aruco", zedPosition->aruco_visible);
    send(message);
}

/** @brief Brief description of function
 * Detailed description of function
 * @param power
 * @return void
 * */
void powerCallback(const messages::msg::Power::SharedPtr power){
    //RCLCPP_INFO(nodeHandle->get_logger(), "power callback");
    send("Power",power);
}

/** @brief Brief description of function
 * Detailed description of function
 * @param talonOut
 * @return void
 * */
void talon1Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon1 callback");
    send("Talon 1",talonOut);
}

/** @brief Brief description of function
 * Detailed description of function
 * @param talonOut
 * @return void
 * */
void talon2Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon2 callback");
    send("Talon 2",talonOut);
}

void talon3Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon3 callback");
    send("Talon 3",talonOut);
}

void talon4Callback(const messages::msg::TalonOut::SharedPtr talonOut){
    //RCLCPP_INFO(nodeHandle->get_logger(), "talon4 callback");
    send("Talon 4",talonOut);
}


void zedImageCallback(const sensor_msgs::msg::Image::SharedPtr inputImage){
    RCLCPP_INFO(nodeHandle->get_logger(), "Received image");
}

/** @brief Brief description of function
 * Detailed description of function
 * @param family
 * @param interfaceName
 * @return addressString
 * */
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


/** @brief Brief description of function
 * Detailed description of function
 * */
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

/** @brief Brief description of function
 * Detailed description of function
 * */
void reboot(){
    sync();
    reboot(LINUX_REBOOT_CMD_POWER_OFF);
}

std::string robotName="unnamed";
bool broadcast=true;
/** @brief Brief description of function
 * Detailed description of function
 * */
void broadcastIP(){
    while(true){
        if(broadcast){
            std::string addressString=getAddressString(AF_INET,"wlan0");

            std::string message(robotName+"@"+addressString);
            std::cout << message << std::endl << std::flush;

            int socketDescriptor=socket(AF_INET, SOCK_DGRAM, 0);

            //if(socket>=0){
            if(socketDescriptor>=0){
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
    rclcpp::init(argc,argv);

    nodeHandle = rclcpp::Node::make_shared("communication2");
    RCLCPP_INFO(nodeHandle->get_logger(),"Starting communication2 node");

    nodeHandle->declare_parameter<std::string>("robot_name","not named");
    rclcpp::Parameter robotNameParameter = nodeHandle->get_parameter("robot_name");
    robotName = robotNameParameter.as_string();
    RCLCPP_INFO(nodeHandle->get_logger(),"robotName: %s", robotName.c_str());

    auto joystickAxisPublisher = nodeHandle->create_publisher<messages::msg::AxisState>("joystick_axis", 1);
    auto joystickHatPublisher = nodeHandle->create_publisher<messages::msg::HatState>("joystick_hat",1);
    auto joystickButtonPublisher = nodeHandle->create_publisher<messages::msg::ButtonState>("joystick_button",1);
    auto keyPublisher = nodeHandle->create_publisher<messages::msg::KeyState>("key",1);
    auto stopPublisher = nodeHandle->create_publisher<std_msgs::msg::Empty>("STOP",1);
    auto goPublisher=nodeHandle->create_publisher<std_msgs::msg::Empty>("GO",1);

    auto powerSubscriber = nodeHandle->create_subscription<messages::msg::Power>("power",1,powerCallback);
    auto talon1Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_14_info",1,talon1Callback);
    auto talon2Subscriber = nodeHandle->create_subscription<messages::msg::TalonOut>("talon_15_info",1,talon2Callback);
    auto zedPositionSubscriber = nodeHandle->create_subscription<messages::msg::ZedPosition>("zed_position",1,zedPositionCallback);
    auto zedImageSubscriber = nodeHandle->create_subscription<sensor_msgs::msg::Image>("zed_image", 10, zedImageCallback);

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
    send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
    silentRunning=true;

    fcntl(new_socket, F_SETFL, O_NONBLOCK);

    std::list<uint8_t> messageBytesList;
    uint8_t message[256];
    rclcpp::Rate rate(20);
    while(rclcpp::ok()){
        bytesRead = recv(new_socket, buffer, 1024, 0);
        for(int index=0;index<bytesRead;index++){
            messageBytesList.push_back(buffer[index]);
        }

        if(bytesRead==0){
            stopPublisher->publish(empty);
	    RCLCPP_INFO(nodeHandle->get_logger(),"Lost Connection");
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
            send(new_socket, hello.c_str(), strlen(hello.c_str()), 0); 
            fcntl(new_socket, F_SETFL, O_NONBLOCK);
    
            silentRunning=true;
        }

        while(messageBytesList.size()>0 && messageBytesList.front()<=messageBytesList.size()){
	    //RCLCPP_INFO(nodeHandle->get_logger(),"bytes read %d", bytesRead);
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
                messages::msg::AxisState axisState;
                axisState.joystick=message[1];
                axisState.axis=message[2];
                axisState.state=parseFloat(&message[3]);
		joystickAxisPublisher->publish(axisState);
		//RCLCPP_INFO(nodeHandle->get_logger(),"axis %d %d %f ", axisState.joystick, axisState.axis , axisState.state);
            }

            if(command==2){
                messages::msg::KeyState keyState;
                keyState.key=((uint16_t)message[1])<<8 | ((uint16_t)message[2]);
                keyState.state=message[3];
                keyPublisher->publish(keyState);
		//RCLCPP_INFO(nodeHandle->get_logger(),"key %d %d ", keyState.key , keyState.state);
            }

            if(command==5){
                messages::msg::ButtonState buttonState;
                buttonState.joystick=message[1];
                buttonState.button=message[2];
                buttonState.state=message[3];
                if(buttonState.button==0 && buttonState.state==0){
                    std::cout << "publish stop" << std::endl;
                    stopPublisher->publish(empty);
                }    
                if(buttonState.button==0 && buttonState.state==1){
                    std::cout << "publish go" << std::endl;
                    goPublisher->publish(empty);
                }    
                joystickButtonPublisher->publish(buttonState);
		//RCLCPP_INFO(nodeHandle->get_logger(),"button %d %d %d", buttonState.joystick , buttonState.button , buttonState.state);
            }
            if(command==6){
                messages::msg::HatState hatState;
                hatState.joystick=message[1];
                hatState.hat=message[2];
                hatState.state=message[3];
                joystickHatPublisher->publish(hatState);
		//RCLCPP_INFO(nodeHandle->get_logger(),"hat %d %d %d", hatState.joystick , hatState.hat , hatState.state);
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

        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }

    broadcastThread.join();
}

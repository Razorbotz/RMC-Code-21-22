#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include "messages/TalonOut.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


bool GO=false;
void stopCallback(std_msgs::Empty empty){
	std::cout << "STOP" << std::endl;
	GO=false;
} 
void goCallback(std_msgs::Empty empty){
	std::cout << "GO" << std::endl;
	GO=true;
}

bool useVelocity=false;
int velocityMultiplier=0;
int testSpeed=0;
TalonSRX* talonSRX;
void speedCallback(const std_msgs::Float32 speed){
	//std::cout << "---------->>>  " << speed.data << std::endl;

	if(useVelocity){
        	talonSRX->Set(ControlMode::Velocity, int(speed.data*velocityMultiplier));
		//talonSRX->Set(ControlMode::Velocity, testSpeed);
	}else{
        	talonSRX->Set(ControlMode::PercentOutput, speed.data);
	}
}


int main(int argc,char** argv){
	std::cout << "Starting talon 3" << std::endl;
	ros::init(argc,argv,"talon_node");
	ros::NodeHandle nodeHandleP("~");
	ros::NodeHandle nodeHandle;

	int success;

	std::string message;
        success=nodeHandleP.getParam("message", message);
        std::cout << success << "message " << message << std::endl;


	int motorNumber=0;
	success=nodeHandleP.getParam("motor_number", motorNumber);
	std::cout << success << "motor_number: " << motorNumber << std::endl;
	
	std::string infoTopic;
	success=nodeHandleP.getParam("info_topic", infoTopic);
	std::cout << success << "info_topic: " << infoTopic << std::endl;

	std::string speedTopic;
	success=nodeHandleP.getParam("speed_topic", speedTopic);
	std::cout << success << "speed_topic: " << speedTopic << std::endl;

	bool invertMotor=false;
	success=nodeHandleP.getParam("invert_motor", invertMotor);
	std::cout << success << "invert_motor: " << invertMotor << std::endl;

	success=nodeHandleP.getParam("use_velocity", useVelocity);
	std::cout << success << "use_velocity: " << useVelocity << std::endl;

	success=nodeHandleP.getParam("velocity_multiplier", velocityMultiplier);
	std::cout << success << "velocity_multiplier: " << velocityMultiplier << std::endl;

	success=nodeHandleP.getParam("test_speed", testSpeed);
	std::cout << success << "test_speed: " << testSpeed << std::endl;

	double kP=0;
	success=nodeHandleP.getParam("kP", kP);
	std::cout << success << "kP: " << kP << std::endl;

	double kI=0;
	success=nodeHandleP.getParam("kI", kI);
	std::cout << success << "kI: " << kI << std::endl;

	double kD=0;
	success=nodeHandleP.getParam("kD", kD);
	std::cout << success << "kD: " << kD << std::endl;

	double kF=0;
	success=nodeHandleP.getParam("kF", kF);
	std::cout << success << "kF: " << kF << std::endl;
	//motorNumber=1;
	//infoTopic="talon_1_info";
	//speedTopic="talon_1_speed";

        ctre::phoenix::platform::can::SetCANInterface("can0");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	int kSlotIdx=0;
	talonSRX=new TalonSRX(motorNumber);
	talonSRX->SetInverted(invertMotor);

	talonSRX->SelectProfileSlot(0,0);
	talonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
	//talonSRX->SetSensorPhase(false);
	talonSRX->ConfigClosedloopRamp(2);
	talonSRX->ConfigNominalOutputForward(0, kTimeoutMs);
	talonSRX->ConfigNominalOutputReverse(0, kTimeoutMs);
	talonSRX->ConfigPeakOutputForward(1, kTimeoutMs);
	talonSRX->ConfigPeakOutputReverse(-1, kTimeoutMs);
	talonSRX->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	talonSRX->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	talonSRX->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
	talonSRX->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
	talonSRX->ConfigAllowableClosedloopError(kPIDLoopIdx,0,kTimeoutMs);

        talonSRX->Set(ControlMode::PercentOutput, 0);
        talonSRX->Set(ControlMode::Velocity, 0);

	TalonSRXPIDSetConfiguration pid;
	TalonSRXConfiguration allConfigs;
	StatusFrame statusFrame;

	messages::TalonOut talonOut;
	ros::Publisher talonOutPublisher=nodeHandle.advertise<messages::TalonOut>(infoTopic.c_str(),1);
	ros::Subscriber speedSubscriber=nodeHandle.subscribe(speedTopic.c_str(),1,speedCallback);

	ros::Subscriber stopSubscriber=nodeHandle.subscribe("STOP",1,stopCallback); 
	ros::Subscriber goSubscriber=nodeHandle.subscribe("GO",1,goCallback); 

	int count=0;
        auto start = std::chrono::high_resolution_clock::now();
        while(ros::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
		auto finish = std::chrono::high_resolution_clock::now();

		if(std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() > 250000000){
			int deviceID=talonSRX->GetDeviceID();
			double busVoltage=talonSRX->GetBusVoltage();
			double outputCurrent=talonSRX->GetOutputCurrent();
			bool isInverted=talonSRX->GetInverted();
			double motorOutputVoltage=talonSRX->GetMotorOutputVoltage();
			double motorOutputPercent=talonSRX->GetMotorOutputPercent();
			double temperature=talonSRX->GetTemperature();
			int sensorPosition0=talonSRX->GetSelectedSensorPosition(0);
			int sensorVelocity0=talonSRX->GetSelectedSensorVelocity(0);
			int closedLoopError0=talonSRX->GetClosedLoopError(0);
			double integralAccumulator0=talonSRX->GetIntegralAccumulator(0);
			double errorDerivative0=talonSRX->GetErrorDerivative(0);
		
			talonOut.deviceID=deviceID;	
			talonOut.busVoltage=busVoltage;
			talonOut.outputCurrent=outputCurrent;
			talonOut.outputVoltage=motorOutputVoltage;
			talonOut.outputPercent=motorOutputPercent;
			talonOut.temperature=temperature;
			talonOut.sensorPosition=sensorPosition0;
			talonOut.sensorVelocity=sensorVelocity0;
			talonOut.closedLoopError=closedLoopError0;
			talonOut.integralAccumulator=integralAccumulator0;
			talonOut.errorDerivative=errorDerivative0;

			talonOutPublisher.publish(talonOut);
        		start = std::chrono::high_resolution_clock::now();
		}

		if(count++>200 && GO){
			std::cout <<"V=" << talonSRX->GetSelectedSensorVelocity(kPIDLoopIdx) <<"  "
				<< "  E=" << talonSRX->GetClosedLoopError(kPIDLoopIdx) 
				<< "  IA=" << talonSRX->GetIntegralAccumulator(kPIDLoopIdx)
				<< "  ED=" << talonSRX->GetErrorDerivative(kPIDLoopIdx) 
				<< std::endl;
			count=0;
		}
		 	
/*
		talonSRX->GetPIDConfigs(pid, 0,0);
		talonSRX->GetAllConfigs(allConfigs);

		int id=talonSRX->GetDeviceID();
		double voltage=talonSRX->GetBusVoltage();
		double outputCurrent=talonSRX->GetOutputCurrent();
		bool isInverted=talonSRX->GetInverted();
		double motorOutputPercent=talonSRX->GetMotorOutputPercent();
		double motorOutputVoltage=talonSRX->GetMotorOutputVoltage();
		double temperature=talonSRX->GetTemperature();
		int sensorPosition0=talonSRX->GetSelectedSensorPosition(0);
		int sensorVelocity0=talonSRX->GetSelectedSensorVelocity(0);
		int cloosedLoopError0=talonSRX->GetClosedLoopError(0);
		double integralAccumulator0=talonSRX->GetIntegralAccumulator(0);
		double errorDerivative0=talonSRX->GetErrorDerivative(0);
		double closedLoopTargeti0=talonSRX->GetClosedLoopTarget(0);
		int activeTrajectoryPosition0=talonSRX->GetActiveTrajectoryPosition(0);
		int activeTrajectoryVelocity0=talonSRX->GetActiveTrajectoryVelocity(0);
		double activeTrajectoryArbFeedFwd0=talonSRX->GetActiveTrajectoryArbFeedFwd(0);
		bool isMotionProfileFinished=talonSRX->IsMotionProfileFinished();
		bool isMotionProfileTopLevelBufferFull=talonSRX->IsMotionProfileTopLevelBufferFull();
		ctre::phoenix::ErrorCode errorCode=talonSRX->GetLastError();
		int firmwareVersion=talonSRX->GetFirmwareVersion();
		bool hasRestOccured=talonSRX->HasResetOccurred();
*/
                usleep(20);
		ros::spinOnce();
        }
}


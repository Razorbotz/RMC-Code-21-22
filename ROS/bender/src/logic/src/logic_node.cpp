#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

#include <messages/ButtonState.h>
#include <messages/HatState.h>

float joystick1Roll=0;
float joystick1Pitch=0;
float joystick1Yaw=0;
float joystick1Throttle=1;
bool drive=true; //toggles driving and excavation
int8_t elbowDir = 1;
float shoulderOffsetAngle = 0;

ros::Publisher driveLeftSpeedPublisher;
ros::Publisher driveRightSpeedPublisher;
ros::Publisher elbowLeftSpeedPublisher;
ros::Publisher elbowRightSpeedPublisher;
ros::Publisher binSpeedPublisher;
ros::Publisher drumSpeedPublisher;
ros::Publisher scoopDirPublisher;
ros::Publisher shoulderSpeedPublisher;
ros::Publisher shoulderLeftGoPublisher;
ros::Publisher shoulderRightGoPublisher;
ros::Publisher driveStatePublisher;


//========================= Drivetrain logic =========================*/
void updateSpeed(){
	
	std_msgs::Float32 speedLeft;
	std_msgs::Float32 speedRight;
	float maxSpeed = 0.4;
	
	//pushes values near 0 to 0 (creates a dead zone)
	joystick1Roll = (fabs(joystick1Roll)<0.1)? 0.0 : joystick1Roll; // if roll in near 0 make it 0
        joystick1Pitch = (fabs(joystick1Pitch)<0.1)? 0.0 : joystick1Pitch; // if pitch in near 0 make it 0


	//Linear transformation of cordinate planes
	speedLeft.data = (joystick1Pitch + joystick1Roll);
	speedRight.data = (joystick1Pitch - joystick1Roll);
	
	//multiplied the output speed by the "throttle" value and a limiting precentage
	speedLeft.data = speedLeft.data * maxSpeed;
	speedRight.data = speedRight.data * maxSpeed;
	
	std::cout << "speed " << speedLeft.data << "   " << speedRight.data << std::endl;

	if(drive){
	    driveLeftSpeedPublisher.publish(speedLeft);
	    driveRightSpeedPublisher.publish(speedRight);
	}
	else{
	    speedLeft.data=0.0;
	    speedRight.data=0.0; 
	    driveLeftSpeedPublisher.publish(speedLeft);
            driveRightSpeedPublisher.publish(speedRight);
        }
}

//============================== Excavation logic ==============================*/
/*
//this is a one time calculation of the offset angle of the lower arm
void calculateShoulderOffsetAngle(){
	// angle_1 = sin inverse off Opposite over Hypotenuse of the bottom angle
	float Hypo = 193.675; //distance between the axis of the actuator mount and the shoulder mount		<-User defined const
	float angle_1 = asin(69.85/Hypo);

	// angle_2 = offset angle the the upper arm (constant and will never change)
	float angle_2 = 32.6/180*3.14159;

	std::cout << "angle = " << angle_1 << " " << angle_2 << " " << angle_1+angle_2 << std::endl;
	shoulderOffsetAngle = angle_1 + angle_2;
} // */


void updateArm(){
	//this will take in the joystick Axis and move the arm with Inverse Kinematics
	
//        std_msgs::Float32 theta; //angle of joystick between 0 and 2pi
//        theta.data = acos(joystick1Roll / sqrt(joystick1Roll*joystick1Roll + joystick1Pitch*joystick1Pitch));
//        theta.data = (joystick1Pitch>0)? theta.data : 2*3.14159-theta.data;	
//        std::cout << "theta " << theta.data << std::endl;
	
        joystick1Pitch = (fabs(joystick1Pitch)<0.1)? 0.0 : joystick1Pitch; // if pitch in near 0 make it 0	
        joystick1Yaw = (fabs(joystick1Yaw)<0.3)? 0.0 : joystick1Yaw; // if pitch in near 0 make it 0

	std::cout << "shoulder " << joystick1Pitch << std::endl;
	std::cout << "elbow " << joystick1Yaw << std::endl;

	std_msgs::Float32 elbowSpeed;
	std_msgs::Float32 shoulderSpeed;

	elbowSpeed.data=joystick1Yaw*elbowDir;	
	shoulderSpeed.data=-joystick1Pitch;
	


	if(!drive){
		elbowLeftSpeedPublisher.publish(elbowSpeed);
		elbowRightSpeedPublisher.publish(elbowSpeed);
		shoulderSpeedPublisher.publish(shoulderSpeed);
        }
        else{
		elbowSpeed.data = 0;
		shoulderSpeed.data = 0;
		elbowLeftSpeedPublisher.publish(elbowSpeed);
		elbowRightSpeedPublisher.publish(elbowSpeed);
//		binSpeedPublisher.publish(elbowSpeed);
		shoulderSpeedPublisher.publish(shoulderSpeed);
        }
}


void updateScoop(const int direction=0){
	std::cout << "Scoop(dir) " << direction << std::endl;
	std_msgs::Int8 dir;
	dir.data = direction;
	//switch(direction){
	//    case -1:
	//	std_msgs::Int8 dir
	scoopDirPublisher.publish(dir);
}
void updateDrum(){
	std_msgs::UInt8 drumSpeed;
	drumSpeed.data = joystick1Throttle*255;
	std::cout << "Drum " << drumSpeed << std::endl;
	drumSpeedPublisher.publish(drumSpeed);
}
void updateBin(const int direction=0){
	std_msgs::Float32 speed;
	speed.data = 0.0;
	if (direction==1)
	    speed.data = 0.4;
	else if (direction ==-1)
	    speed.data = -0.4;
	binSpeedPublisher.publish(speed);
}

//============================== joystick1 Axis 1-4 Call backs ===================================*/


void joystick1RollCallback(const std_msgs::Float32& roll){
	//std::cout << "roll " << -roll.data << std::endl;
	joystick1Roll=-roll.data;
	if (drive) updateSpeed();
}
void joystick1PitchCallback(const std_msgs::Float32& pitch){
	//std::cout << "pitch " << pitch.data << std::endl;
	joystick1Pitch=pitch.data;
	if (drive) updateSpeed();
	if (!drive) updateArm();
}
void joystick1YawCallback(const std_msgs::Float32& yaw){	
        //std::cout << "yaw " << yaw.data << std::endl;
	joystick1Yaw=yaw.data;
	if (!drive) updateArm();
}
void joystick1ThrottleCallback(const std_msgs::Float32& throttle){
	//std::cout << "throttle " << throttle.data/2+0.5 << std::endl;
	joystick1Throttle=throttle.data/2+0.5;
	updateDrum();
	
}

//============================== joystick1 Button 1-12 Call backs ===================================*/


void joystick1ButtonCallback(const messages::ButtonState& buttonState){
        std::cout << "Button " << buttonState.number << " " << buttonState.state << std::endl;
	std_msgs::Bool but7;
	std_msgs::Bool but8;
	std_msgs::Bool driveState;

	switch (buttonState.number)
	{
	    case 0: //ESTOP
//		DO NOT USE!!!
		break;
	    case 1: //toggles driving and digging
		if (buttonState.state){
		    drive = !drive;
		    driveState.data = drive;
		    driveStatePublisher.publish(driveState);
//		    std::cout << "drive " << drive << std::endl;
		    updateSpeed();
		    updateArm();
		}
		break;
	    case 2://open Scoop
                if(buttonState.state)
                    updateScoop(-1);
                else
                    updateScoop(0);
                break;
	    case 3: //lowers bin
		if(buttonState.state)
		    updateBin(-1);
		else
		    updateBin(0);
                break;
	    case 4://open Scoop
                if(buttonState.state)
                    updateScoop(1);
                else
                    updateScoop(0);
                break;
	    case 5: //raises bin
		if(buttonState.state)
                    updateBin(1);
                else
                    updateBin(0);
                break;
	    case 6: //actuate left shoulder only
		but7.data = buttonState.state;
		shoulderLeftGoPublisher.publish(but7);
//		std::cout << "BUTTON 7" << std::endl;
                break;
	    case 7: //actuate right shuolder only
		but8.data = buttonState.state;
		shoulderRightGoPublisher.publish(but8);
//		std::cout << "BUTTON 8" << std::endl;
                break;
	    case 8:
                break;
	    case 9:
		if(buttonState.state)
		    elbowDir=elbowDir*-1;
                break;
	    case 10:
                break;
	    case 11:
                break;
	}
}
void joystick1HatCallback(const messages::HatState& hatState){
        std::cout << "Hat " << hatState.state << std::endl;
}

//============================================ main =================================================*/


int main(int argc, char **argv){
	ros::init(argc,argv,"logic");
	ros::NodeHandle nodeHandle;
	ros::Subscriber joystick1RollSubscriber=nodeHandle.subscribe("joystick_1_roll",1,joystick1RollCallback);
	ros::Subscriber joystick1PitchSubscriber=nodeHandle.subscribe("joystick_1_pitch",1,joystick1PitchCallback);
	ros::Subscriber joystick1YawSubscriber=nodeHandle.subscribe("joystick_1_yaw",1,joystick1YawCallback);
	ros::Subscriber joystick1ThrottleSubscriber=nodeHandle.subscribe("joystick_1_throttle",1,joystick1ThrottleCallback);
	ros::Subscriber joystick1ButtonSubscriber=nodeHandle.subscribe("joystick_1_button",1,joystick1ButtonCallback);
	ros::Subscriber joystick1HatSubscriber=nodeHandle.subscribe("joystick_1_hat",1,joystick1HatCallback);
	
	driveLeftSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("drive_left_speed",1);
	driveRightSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("drive_right_speed",1);
	elbowLeftSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("elbow_left_speed",1);
	elbowRightSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("elbow_right_speed",1);
	binSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("bin_speed",1);
	drumSpeedPublisher=nodeHandle.advertise<std_msgs::UInt8>("drum_speed",1);	
        scoopDirPublisher=nodeHandle.advertise<std_msgs::Int8>("scoop_dir",1);
	shoulderSpeedPublisher=nodeHandle.advertise<std_msgs::Float32>("shoulder_speed",1);
	shoulderLeftGoPublisher=nodeHandle.advertise<std_msgs::Bool>("shoulder_L_Go",1);
        shoulderRightGoPublisher=nodeHandle.advertise<std_msgs::Bool>("shoulder_R_Go",1);
	driveStatePublisher=nodeHandle.advertise<std_msgs::Bool>("drive_state",1);


	//pre calculations
//	calculateShoulderOffsetAngle();

	ros::Rate rate(10);
	double increment=.01;
	double joystickXPrevious=0;
	while(ros::ok()){
		ros::spinOnce();
	}
}

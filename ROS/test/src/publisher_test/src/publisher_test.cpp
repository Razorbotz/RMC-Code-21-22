#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"publisher_test");
	ros::NodeHandle nodeHandle;
	ros::Publisher publisher=nodeHandle.advertise<std_msgs::Float64>("topic1",1);
	std_msgs::Float64 inputFloat;

	inputFloat.data=0.0;

	ros::Rate rate(1);

	while(ros::ok()){
		inputFloat.data = inputFloat.data + 0.001;
		publisher.publish(inputFloat);
		rate.sleep();
	}
}

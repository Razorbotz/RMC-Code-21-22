#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <messages/Power.h>

void callback(const std_msgs::Float64 messageHolder){
	ROS_INFO("received value is: %f", messageHolder.data);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"minimal_subscriber");
	ros::NodeHandle nodeHandle;
	ros::Subscriber subscriber=nodeHandle.subscribe("topic1",1,callback);
	
	ros::spin();

	return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
ros::Publisher drive_pub;

void driveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	std::vector<double> inp = msg -> data;
	float Actuator1 = inp[3];
	float Actuator2 = inp[4];
	//float angSpeed = inp[1]; 

	std::vector<double> out(2, 0);
	out[0] = Actuator1;
	out[1] = Actuator2;

	std_msgs::Float64MultiArray outMsg;
	outMsg.data = out;
	drive_pub.publish(outMsg);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "mobility_driver");
	ros::NodeHandle _nh;
	drive_pub = _nh.advertise<std_msgs::Float64MultiArray>("/rover/drive_directives", 100);
	ros::Subscriber drive_sub = _nh.subscribe("/rover/control_directives", 100, driveCallback);
	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}
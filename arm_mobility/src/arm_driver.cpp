#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
ros::Publisher arm_pub;

void driveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	std::vector<double> inp = msg -> data;
	float BaseRotn=inp[2];
	float ShoulderActuator = inp[3];
	float ElbowActuator = inp[4];
	float WristActuator=inp[5];
	float FingerActuator=inp[6];
	float GripperMotor=inp[7];
	//float angSpeed = inp[1]; 

	std::vector<double> out(6, 0);
	out[0]=BaseRotn;
	out[1]=ShoulderActuator;
	out[2]=ElbowActuator;
	out[3]=WristActuator;
	out[4]=FingerActuator;
	out[5]=GripperMotor;

	std_msgs::Float64MultiArray outMsg;
	outMsg.data = out;
	arm_pub.publish(outMsg);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "mobility_driver");
	ros::NodeHandle _nh;
	arm_pub = _nh.advertise<std_msgs::Float64MultiArray>("/rover/arm_directives", 100);
	ros::Subscriber drive_sub = _nh.subscribe("/rover/control_directives", 100, driveCallback);
	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}
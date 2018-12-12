#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Subscriber joysub;
ros::Publisher referens;

geometry_msgs::Twist ref;

float vmax;
float wmax;

float speedMap(float input)
{
	return input * vmax / 2; // mapt to desierd speed in m/s
}

// adjust input sensitivity
float inputSens(float input){
	if (input >= 0)
		return pow(input, 2) * wmax;
	else
		return -pow(input,2) * wmax;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	ref.linear.x = speedMap(-msg->axes[5] + msg->axes[2]); // -forword_speed + bacvards speed => (-2,2)
	ref.angular.z = inputSens(msg->axes[0]); 

	if (ref.linear.x < 0) ref.angular.z = -ref.angular.z;	// adjust stering if in revers
	referens.publish(ref);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "referens_node");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	
	nh.param<float>("max_velosety", vmax, 1); // 1.2 theoretic max	
	nh.param<float>("max_angular_velosety", wmax, 1); // 8.4 down scaild to macht control loop

	referens = n.advertise<geometry_msgs::Twist>("referens", 5);
	joysub = n.subscribe<sensor_msgs::Joy>("joy", 5, joyCallback);
	
	ref.linear.x = 0;
	ref.linear.y = 0;
	ref.linear.z = 0;
	ref.angular.x = 0;
	ref.angular.y = 0;
	ref.angular.z = 0;

	ros::spin();

	return 0;
}

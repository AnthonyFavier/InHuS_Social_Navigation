#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub_cmd_vel;

void finalCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub_cmd_vel.publish(*msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "output_interface");
	ros::NodeHandle nh;

	ros::Subscriber sub_final_cmd_vel = nh.subscribe("out/final_cmd", 10, finalCmdCB);
	pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/stage_agents/human1/cmd_vel", 10);

	ros::spin();

	return 0;
}

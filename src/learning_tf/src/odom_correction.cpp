#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

ros::Publisher pub_corr_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry new_odom;
	new_odom = *msg;

	new_odom.twist.twist.linear.x = sqrt(pow(new_odom.twist.twist.linear.x,2) + pow(new_odom.twist.twist.linear.y,2));
	new_odom.twist.twist.linear.y = 0;

	pub_corr_odom.publish(new_odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_correction");

	ros::NodeHandle nh;

	pub_corr_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);
	ros::Subscriber sub_odom = nh.subscribe("raw_odom", 100, odomCallback);

	ros::spin();

	return 0;
}

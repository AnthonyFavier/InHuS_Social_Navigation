#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

ros::Publisher pub_sim_human_pose;
ros::Publisher pub_corr_odom;

geometry_msgs::Pose2D h_pose;
double roll, pitch, yaw;

void humanPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	h_pose.theta=yaw;
	h_pose.x=msg->pose.position.x;
	h_pose.y=msg->pose.position.y;

	pub_sim_human_pose.publish(h_pose);
}

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
	ros::init(argc, argv, "preprocessing");
	ros::NodeHandle nh;

	ros::Subscriber sub_human_pose = nh.subscribe("human/pose", 100, humanPoseCallback);
	pub_sim_human_pose = nh.advertise<geometry_msgs::Pose2D>("sim/human_pose", 100);

	ros::Subscriber sub_odom = nh.subscribe("human/odom", 100, odomCallback);
	pub_corr_odom = nh.advertise<nav_msgs::Odometry>("sim/human_odom", 100);

	ros::spin();

	return 0;
}

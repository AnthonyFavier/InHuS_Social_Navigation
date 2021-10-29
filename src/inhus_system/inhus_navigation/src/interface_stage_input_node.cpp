#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "inhus/PoseVel.h"

// Get inputs from the simulator
// Only change "Input Calbacks" and "Input Subscribers"
// to adapt to another simulator

// Publish tf human and robot frame in map frame
// according to the inputs from the simulator

ros::Publisher pub_sim_human_odom;
ros::Publisher pub_sim_human_pose_vel;
inhus::PoseVel h_pose_vel;

ros::Publisher pub_sim_robot_pose_vel;
inhus::PoseVel r_pose_vel;

/////////////////////////
//// Input Callbacks //// Part to modify for another simulator
/////////////////////////
void humanOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double h_roll, h_pitch, h_yaw;
	m.getRPY(h_roll, h_pitch, h_yaw);
	h_pose_vel.pose.theta=h_yaw;
	h_pose_vel.pose.x=msg->pose.pose.position.x;
	h_pose_vel.pose.y=msg->pose.pose.position.y;
	h_pose_vel.vel = msg->twist.twist;

	pub_sim_human_pose_vel.publish(h_pose_vel);
	pub_sim_human_odom.publish(*msg);
}

void robotOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double r_roll, r_pitch, r_yaw;
	m.getRPY(r_roll, r_pitch, r_yaw);
	r_pose_vel.pose.theta=r_yaw;
	r_pose_vel.pose.x=msg->pose.pose.position.x;
	r_pose_vel.pose.y=msg->pose.pose.position.y;
	r_pose_vel.vel = msg->twist.twist;

	pub_sim_robot_pose_vel.publish(r_pose_vel);
}
/////////////////////////
/////////////////////////
/////////////////////////


int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_interface");
	ros::NodeHandle nh;

	////////////////////////////
	////  Input Subscribers //// Part to modify for another simulator
  	////////////////////////////
	ros::Subscriber sub_human_odom = nh.subscribe("/stage_agents/human1/odom", 10, humanOdomCallback);
	ros::Subscriber sub_robot_odom = nh.subscribe("/stage_agents/pr2/odom", 10, robotOdomCallback);
	////////////////////////////
	////////////////////////////
	////////////////////////////

	////  Ouput Publishers  ////
	pub_sim_human_pose_vel = nh.advertise<inhus::PoseVel>("in/human_pose_vel", 100);
	pub_sim_human_odom = nh.advertise<nav_msgs::Odometry>("in/human_odom", 100);
	pub_sim_robot_pose_vel = nh.advertise<inhus::PoseVel>("in/robot_pose_vel", 100);
	////////////////////////////

	tf::TransformBroadcaster br;
	tf::Quaternion q;
	tf::Transform tf_map_robot;

	ros::Rate rate(30);

	while(ros::ok())
	{
		tf_map_robot.setOrigin(tf::Vector3(r_pose_vel.pose.x, r_pose_vel.pose.y, 0));
		q.setRPY(0, 0, r_pose_vel.pose.theta);
		tf_map_robot.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_map_robot, ros::Time::now(), "map", "robot_inhus"));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

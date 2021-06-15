#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_broadcaster.h>

// Get inputs from the simulator
// Only change "Input Calbacks" and "Input Subscribers"
// to adapt to another simulator

// Publish tf human and robot frame in map frame
// according to the inputs from the simulator

ros::Publisher pub_sim_human_pose;
geometry_msgs::Pose2D h_pose;
double h_roll, h_pitch, h_yaw;

ros::Publisher pub_sim_human_vel;
geometry_msgs::Twist h_vel;

ros::Publisher pub_sim_robot_pose;
geometry_msgs::Pose2D r_pose;
double r_roll, r_pitch, r_yaw;

ros::Publisher pub_sim_robot_vel;
geometry_msgs::Twist r_vel;

/////////////////////////
//// Input Callbacks //// Part to modify for another simulator
/////////////////////////
void humanPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(h_roll, h_pitch, h_yaw);
	h_pose.theta=h_yaw;
	h_pose.x=msg->pose.position.x;
	h_pose.y=msg->pose.position.y;

	pub_sim_human_pose.publish(h_pose);
}

void humanVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	h_vel = msg->twist;
	pub_sim_human_vel.publish(h_vel);
}

void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(r_roll, r_pitch, r_yaw);
	r_pose.theta=r_yaw;
	r_pose.x=msg->pose.position.x;
	r_pose.y=msg->pose.position.y;

	pub_sim_robot_pose.publish(r_pose);
}

void robotVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	r_vel = msg->twist;
	pub_sim_robot_vel.publish(r_vel);
}
/////////////////////////
/////////////////////////
/////////////////////////


int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_interface");
	ros::NodeHandle nh;

	tf::TransformBroadcaster br;
	tf::Quaternion q;

	tf::Transform tf_map_footprint;
	tf::Transform tf_map_robot;

	ros::Rate rate(30);

	////////////////////////////
	////  Input Subscribers //// Part to modify for another simulator
  ////////////////////////////
	ros::Subscriber sub_human_pose = nh.subscribe("/morse/human1_pose", 100, humanPoseCallback);
	ros::Subscriber sub_human_vel = nh.subscribe("/morse/human1_vel", 100, humanVelCallback);
	ros::Subscriber sub_robot_pose = nh.subscribe("/morse/robot_pose", 100, robotPoseCallback);
	ros::Subscriber sub_robot_vel = nh.subscribe("/morse/robot_vel", 100, robotVelCallback);
	////////////////////////////
	////////////////////////////
	////////////////////////////

	////  Ouput Publishers  ////
	pub_sim_human_pose = nh.advertise<geometry_msgs::Pose2D>("in/human_pose", 100);
	pub_sim_human_vel = nh.advertise<geometry_msgs::Twist>("in/human_vel", 100);
	pub_sim_robot_pose = nh.advertise<geometry_msgs::Pose2D>("in/robot_pose", 100);
	pub_sim_robot_vel = nh.advertise<geometry_msgs::Twist>("in/robot_vel", 100);
	////////////////////////////

	while(ros::ok())
	{
		tf_map_footprint.setOrigin(tf::Vector3(h_pose.x, h_pose.y, 0));
		q.setRPY(0, 0, h_pose.theta);
		tf_map_footprint.setRotation(q);

		tf_map_robot.setOrigin(tf::Vector3(r_pose.x, r_pose.y, 0));
		q.setRPY(0, 0, r_pose.theta);
		tf_map_robot.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_map_footprint, ros::Time::now(), "human1/odom", "human1/base_footprint"));
		//br.sendTransform(tf::StampedTransform(tf_map_robot, ros::Time::now(), "map", "base_footprint"));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

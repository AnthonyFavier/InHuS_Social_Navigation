#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_broadcaster.h>

// Convert geometry_msgs::PoseStamped messages from MORSE
// to geometry_msgs::Pose2D messages

// Publish tf human and robot frame in map frame

ros::Publisher pub_sim_human_pose;
geometry_msgs::Pose2D h_pose;
double h_roll, h_pitch, h_yaw;

ros::Publisher pub_sim_human_vel;
geometry_msgs::Twist h_vel;

ros::Publisher pub_sim_human_other_pose;
geometry_msgs::Pose2D h_o_pose;
double h_o_roll, h_o_pitch, h_o_yaw;

ros::Publisher pub_sim_human_other_vel;
geometry_msgs::Twist h_o_vel;

ros::Publisher pub_sim_robot_pose;
geometry_msgs::Pose2D r_pose;
double r_roll, r_pitch, r_yaw;

ros::Publisher pub_sim_robot_vel;
geometry_msgs::Twist r_vel;

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

void humanOtherPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(h_o_roll, h_o_pitch, h_o_yaw);
	h_o_pose.theta=h_yaw;
	h_o_pose.x=msg->pose.position.x;
	h_o_pose.y=msg->pose.position.y;

	pub_sim_human_other_pose.publish(h_o_pose);
}

void humanOtherVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	h_o_vel = msg->twist;
	pub_sim_human_other_vel.publish(h_o_vel);
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_interface");
	ros::NodeHandle nh;

	ros::Rate rate(30);

	ros::Subscriber sub_human_pose = nh.subscribe("/morse/human2_pose", 100, humanPoseCallback);
	pub_sim_human_pose = nh.advertise<geometry_msgs::Pose2D>("in/human_pose", 100);

	ros::Subscriber sub_human_vel = nh.subscribe("/morse/human2_vel", 100, humanVelCallback);
	pub_sim_human_vel = nh.advertise<geometry_msgs::Twist>("in/human_vel", 100);

	ros::Subscriber sub_human_other_pose = nh.subscribe("/morse/human_pose", 100, humanOtherPoseCallback);
	pub_sim_human_other_pose = nh.advertise<geometry_msgs::Pose2D>("in/human_other_pose", 100);

	ros::Subscriber sub_human_other_vel = nh.subscribe("/morse/human_vel", 100, humanOtherVelCallback);
	pub_sim_human_other_vel = nh.advertise<geometry_msgs::Twist>("in/human_other_vel", 100);

	ros::Subscriber sub_robot_pose = nh.subscribe("/morse/robot_pose", 100, robotPoseCallback);
	pub_sim_robot_pose = nh.advertise<geometry_msgs::Pose2D>("in/robot_pose", 100);

	ros::Subscriber sub_robot_vel = nh.subscribe("/morse/robot_vel", 100, robotVelCallback);
	pub_sim_robot_vel = nh.advertise<geometry_msgs::Twist>("in/robot_vel", 100);

	tf::TransformBroadcaster br;
	tf::Quaternion q;

	tf::Transform tf_map_footprint;
	tf::Transform tf_map_robot;

	while(ros::ok())
	{
		tf_map_footprint.setOrigin(tf::Vector3(h_pose.x, h_pose.y, 0));
		q.setRPY(0, 0, h_pose.theta);
		tf_map_footprint.setRotation(q);

		tf_map_robot.setOrigin(tf::Vector3(r_pose.x, r_pose.y, 0));
		q.setRPY(0, 0, r_pose.theta);
		tf_map_robot.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_map_footprint, ros::Time::now(), "map", "human2_human_base"));
		br.sendTransform(tf::StampedTransform(tf_map_robot, ros::Time::now(), "map", "human2_robot_base"));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

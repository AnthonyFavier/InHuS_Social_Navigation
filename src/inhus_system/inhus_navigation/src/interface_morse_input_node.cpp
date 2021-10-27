#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "cohan_msgs/AgentMarkerStamped.h"

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
void humanCallback(const cohan_msgs::AgentMarkerStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->agent.pose.orientation.x,msg->agent.pose.orientation.y,msg->agent.pose.orientation.z,msg->agent.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(h_roll, h_pitch, h_yaw);
	h_pose.theta=h_yaw;
	h_pose.x=msg->agent.pose.position.x;
	h_pose.y=msg->agent.pose.position.y;

	h_vel = msg->agent.velocity;

	pub_sim_human_pose.publish(h_pose);
	pub_sim_human_vel.publish(h_vel);
}

void robotCallback(const cohan_msgs::AgentMarkerStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->agent.pose.orientation.x,msg->agent.pose.orientation.y,msg->agent.pose.orientation.z,msg->agent.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(r_roll, r_pitch, r_yaw);
	r_pose.theta=r_yaw;
	r_pose.x=msg->agent.pose.position.x;
	r_pose.y=msg->agent.pose.position.y;

	r_vel = msg->agent.velocity;

	pub_sim_robot_pose.publish(r_pose);
	pub_sim_robot_vel.publish(r_vel);
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
	ros::Subscriber sub_human_odom = nh.subscribe("/human1", 10, humanCallback);
	ros::Subscriber sub_robot_odom = nh.subscribe("/pr2_pose_vel", 10, robotCallback);
	////////////////////////////
	////////////////////////////
	////////////////////////////

	////  Ouput Publishers  ////
	pub_sim_human_pose = nh.advertise<geometry_msgs::Pose2D>("in/human_pose", 100);
	pub_sim_human_vel = nh.advertise<geometry_msgs::Twist>("in/human_vel", 100);
	pub_sim_robot_pose = nh.advertise<geometry_msgs::Pose2D>("in/robot_pose", 100);
	pub_sim_robot_vel = nh.advertise<geometry_msgs::Twist>("in/robot_vel", 100);
	////////////////////////////

	tf::TransformBroadcaster br;
	tf::Quaternion q;
	tf::Transform tf_map_robot;

	ros::Rate rate(30);

	while(ros::ok())
	{
		tf_map_robot.setOrigin(tf::Vector3(r_pose.x, r_pose.y, 0));
		q.setRPY(0, 0, r_pose.theta);
		tf_map_robot.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_map_robot, ros::Time::now(), "map", "robot_inhus"));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

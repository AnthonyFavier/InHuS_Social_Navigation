#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_sim_human_pose;
geometry_msgs::Pose2D h_pose;
double h_roll, h_pitch, h_yaw;

ros::Publisher pub_sim_robot_pose;
geometry_msgs::Pose2D r_pose;
double r_roll, r_pitch, r_yaw;

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "preprocessing");
	ros::NodeHandle nh;

	ros::Rate rate(30);

	ros::Subscriber sub_human_pose = nh.subscribe("/morse/human_pose", 100, humanPoseCallback);
	pub_sim_human_pose = nh.advertise<geometry_msgs::Pose2D>("sim/human_pose", 100);

	ros::Subscriber sub_robot_pose = nh.subscribe("/morse/robot_pose", 100, robotPoseCallback);
	pub_sim_robot_pose = nh.advertise<geometry_msgs::Pose2D>("sim/robot_pose", 100);

	/*tf::TransformBroadcaster br;
	tf::Quaternion q;

	tf::Transform tf_map_footprint;
	tf::Transform tf_map_robot;*/

	while(ros::ok())
	{
		/*tf_map_footprint.setOrigin(tf::Vector3(h_pose.x, h_pose.y, 0));
		q.setRPY(0, 0, h_pose.theta);
		tf_map_footprint.setRotation(q);

		tf_map_robot.setOrigin(tf::Vector3(r_pose.x, r_pose.y, 0));
		q.setRPY(0, 0, r_pose.theta);
		tf_map_robot.setRotation(q);

		br.sendTransform(tf::StampedTransform(tf_map_footprint, ros::Time::now(), "map", "base_r_footprint"));
		br.sendTransform(tf::StampedTransform(tf_map_robot, ros::Time::now(), "map", "base_r_robot"));*/

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

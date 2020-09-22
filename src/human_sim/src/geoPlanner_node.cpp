#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <human_sim/HActionAction.h>
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

typedef actionlib::SimpleActionServer<human_sim::HActionAction> Server;

Pose2D human_pose;
Pose2D robot_pose;
ros::Publisher pub_cmd;

#define DELTA 0.5

void execute(const human_sim::HActionGoalConstPtr& goal, Server* as)
{
	printf("type=%s destination= (%f, %f, %f)\n", goal->type.c_str(), goal->destination.x, goal->destination.y, goal->destination.theta);
	printf("action on the way\n");
	ros::Rate rate(3);
	geometry_msgs::Twist cmd;

	while(ros::ok() && ((human_pose.x<goal->destination.x-DELTA || human_pose.x>goal->destination.x+DELTA)
		|| (human_pose.y<goal->destination.y-DELTA || human_pose.y>goal->destination.y+DELTA)))
	{
		printf("human_pose = (%f, %f, %f)\n", human_pose.x, human_pose.y, human_pose.theta);
		pub_cmd.publish(cmd);
		rate.sleep();
	}
	printf("human_pose = (%f, %f, %f)\n", human_pose.x, human_pose.y, human_pose.theta);
	as->setSucceeded();
	printf("over\n");
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	human_pose.x=msg->x;
	human_pose.y=msg->y;
	human_pose.theta=msg->theta;
}

void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	robot_pose.x=msg->x;
	robot_pose.y=msg->y;
	robot_pose.theta=msg->theta;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "geoPlanner");
	ros::NodeHandle nh;

	ros::Subscriber sub_pose = nh.subscribe("human_model/human_pose", 100, poseCallback);
	ros::Subscriber sub_robot_pose = nh.subscribe("human_model/robot_pose", 100, robotPoseCallback);
	pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_geo", 100);

	Server server(nh, "do_action", boost::bind(&execute, _1, &server), false);
	server.start();
	printf("action server is on\n");

	ros::spin();

	return 0;
}

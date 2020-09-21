#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <human_sim/ActionHAction.h>
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

typedef actionlib::SimpleActionServer<human_sim::ActionHAction> Server;

Pose2D human_pos;
Pose2D robot_pos;
ros::Publisher pub_cmd;

void execute(const human_sim::ActionHGoalConstPtr& goal, Server* as)
{
	printf("type=%s destination= (%f, %f, %f)\n", goal->type.c_str(), goal->destination.x, goal->destination.y, goal->destination.theta);
	printf("action on the way\n");
	ros::Rate rate(3);
	geometry_msgs::Twist cmd;

	while(ros::ok() && (human_pos.x!=goal->destination.x
		|| human_pos.y!=goal->destination.y
		|| human_pos.theta!=goal->destination.theta))
	{
		printf("human_pos = (%f, %f, %f)\n", human_pos.x, human_pos.y, human_pos.theta);
		pub_cmd.publish(cmd);
		rate.sleep();
	}
	printf("human_pos = (%f, %f, %f)\n", human_pos.x, human_pos.y, human_pos.theta);
	as->setSucceeded();
	printf("over\n");
}

void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	human_pos.x=msg->x;
	human_pos.y=msg->y;
	human_pos.theta=msg->theta;
}

void robotPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	robot_pos.x=msg->x;
	robot_pos.y=msg->y;
	robot_pos.theta=msg->theta;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "geoPlanner");
	ros::NodeHandle nh;

	ros::Subscriber sub_pos = nh.subscribe("human_model/human_pos", 100, posCallback);
	ros::Subscriber sub_robot_pos = nh.subscribe("human_model/robot_pos", 100, robotPosCallback);
	pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_geo", 100);

	Server server(nh, "do_action", boost::bind(&execute, _1, &server), false);
	server.start();
	printf("server on\n");

	ros::spin();

	return 0;
}

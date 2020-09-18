#include "ros/ros.h"
#include <human_sim/DoDishesAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<human_sim::DoDishesAction> Server;

void execute(const human_sim::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
	// Do lots of awesome groundbreaking robot stuff here
	printf("id=%d begin\n",goal->dishwasher_id);
	if(as->isActive())
		printf("Dishes on the way\n");
	ros::Duration(2).sleep();
	as->setSucceeded();
	printf("id=%d over\n",goal->dishwasher_id);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "do_dishes_server");
	ros::NodeHandle n;
	Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
	server.start();
	ros::spin();

	return 0;
}


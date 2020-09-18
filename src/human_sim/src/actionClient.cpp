#include "ros/ros.h"
#include <human_sim/DoDishesAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<human_sim::DoDishesAction> Client;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_client");

	Client client("do_dishes", true);
	client.waitForServer();
	human_sim::DoDishesGoal goal;
	human_sim::DoDishesGoal goal2;
	goal.dishwasher_id=1;
	goal2.dishwasher_id=2;
	printf("Current State: %s\n", client.getState().toString().c_str());

	client.sendGoal(goal);
	printf("Current State: %s\n", client.getState().toString().c_str());

	while(client.getState() != actionlib::SimpleClientGoalState::ACTIVE){}
	printf("Current State: %s\n", client.getState().toString().c_str());

	client.sendGoal(goal2);
	printf("Current State: %s\n", client.getState().toString().c_str());

	client.waitForResult(ros::Duration(5.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Yay! The dishes are now clean\n");
	printf("Current State: %s\n", client.getState().toString().c_str());

	return 0;
}

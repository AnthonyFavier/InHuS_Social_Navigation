#include "taskPlanner.h"

/////////////////// TASK PLANNER /////////////////////

TaskPlanner::TaskPlanner(ros::NodeHandle nh)
{
	nh_=nh;
	service = nh_.advertiseService("compute_plan", &TaskPlanner::computePlan, this);

	printf("compute_plan service is on\n");
}

bool TaskPlanner::computePlan(human_sim::ComputePlan::Request& req, human_sim::ComputePlan::Response& res)
{
	human_sim::ActionHGoal action;

	if(req.type=="Position")
		action.type = "movement";
	else
		action.type = "unknown";
	action.destination.x =		req.x;
	action.destination.y =		req.y;
	action.destination.theta =	req.theta;
	res.actions.push_back(action);

	action.destination.y =		req.y+1;
	res.actions.push_back(action);
}

/////////////////////// MAIN /////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "taskPlanner");

	ros::NodeHandle nh;

	TaskPlanner task_planner(nh);

	ros::spin();

	return 0;
}

//////////////////////////////////////////////////////

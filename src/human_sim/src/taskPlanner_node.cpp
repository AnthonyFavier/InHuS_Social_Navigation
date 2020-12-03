#include "taskPlanner.h"

/////////////////// TASK PLANNER /////////////////////

TaskPlanner::TaskPlanner()
{
	pub_log_ =	nh_.advertise<std_msgs::String>("log", 100);

	service_ = nh_.advertiseService("compute_plan", &TaskPlanner::computePlan, this);
	printf("compute_plan service is on\n");
}

bool TaskPlanner::computePlan(human_sim::ComputePlan::Request& req, human_sim::ComputePlan::Response& res)
{
	move_base_msgs::MoveBaseGoal action;

	action.target_pose.header.frame_id = "map";

	action.target_pose.header.stamp = ros::Time::now();
	action.target_pose.pose.position.x =		req.goal.x;
	action.target_pose.pose.position.y =		req.goal.y;
	tf2::Quaternion q;
	q.setRPY(0,0,req.goal.theta);
	action.target_pose.pose.orientation.x =	q.x();
	action.target_pose.pose.orientation.y =	q.y();
	action.target_pose.pose.orientation.z =	q.z();
	action.target_pose.pose.orientation.w =	q.w();
	res.actions.push_back(action);

	printf("plan done !\n");

	return true;
}

/////////////////////// MAIN /////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "taskPlanner");

	TaskPlanner task_planner;

	ros::spin();

	return 0;
}

//////////////////////////////////////////////////////

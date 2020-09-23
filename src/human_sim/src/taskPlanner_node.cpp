#include "taskPlanner.h"

/////////////////// TASK PLANNER /////////////////////

TaskPlanner::TaskPlanner(ros::NodeHandle nh)
{
	nh_=nh;

	sub_human_pose_ = nh_.subscribe("human_model/human_pose", 100, &TaskPlanner::humanPoseCallback, this);

	service_ = nh_.advertiseService("compute_plan", &TaskPlanner::computePlan, this);
	printf("compute_plan service is on\n");
}

bool TaskPlanner::computePlan(human_sim::ComputePlan::Request& req, human_sim::ComputePlan::Response& res)
{
	human_sim::HActionGoal action;

	if(req.goal.type=="Position")
		action.type = "movement";
	else
		action.type = "unknown";

	action.destination.x =		human_pose_.x+(req.goal.x-human_pose_.x)/2;
	action.destination.y =		human_pose_.y+(req.goal.y-human_pose_.y)/2;
	action.destination.theta =	human_pose_.theta+(req.goal.theta-human_pose_.theta)/2;
	res.actions.push_back(action);

	action.destination.x =		req.goal.x;
	action.destination.y =		req.goal.y;
	action.destination.theta =	req.goal.theta;
	res.actions.push_back(action);

	printf("plan done !\n");
}

void TaskPlanner::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	human_pose_.x=msg->x;
	human_pose_.y=msg->y;
	human_pose_.theta=msg->theta;
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

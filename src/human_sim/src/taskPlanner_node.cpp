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
	move_base_msgs::MoveBaseGoal action;

	action.target_pose.header.frame_id = "map";

	action.target_pose.header.stamp = ros::Time::now();
	//action.target_pose.pose.position.x =		human_pose_.x+(req.goal.x-human_pose_.x)/2;
	//action.target_pose.pose.position.y =		human_pose_.y+(req.goal.y-human_pose_.y)/2;
	action.target_pose.pose.position.x =		req.goal.x;
	action.target_pose.pose.position.y =		req.goal.y;
	tf2::Quaternion q;
	q.setRPY(0,0,req.goal.theta);
	action.target_pose.pose.orientation.x =	q.x();
	action.target_pose.pose.orientation.y =	q.y();
	action.target_pose.pose.orientation.z =	q.z();
	action.target_pose.pose.orientation.w =	q.w();
	res.actions.push_back(action);

	/*action.target_pose.header.stamp = ros::Time::now();
	action.target_pose.pose.position.x =		req.goal.x;
	action.target_pose.pose.position.y =		req.goal.y;
	res.actions.push_back(action);*/

	printf("plan done !\n");

	return true;
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

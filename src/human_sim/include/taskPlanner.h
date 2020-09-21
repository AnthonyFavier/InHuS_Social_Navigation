#ifndef DEF_TASKPLANNER
#define DEF_TASKPLANNER

#include "ros/ros.h"
#include "human_sim/ComputePlan.h"
#include "geometry_msgs/Pose2D.h"
#include "human_sim/ActionHAction.h"
//#include "human_sim/Goal.h"

class TaskPlanner
{
public:
	TaskPlanner(ros::NodeHandle nh);
	bool computePlan(human_sim::ComputePlan::Request& req, human_sim::ComputePlan::Response& res);
private:
	ros::NodeHandle nh_;
	ros::ServiceServer service;
};

#endif

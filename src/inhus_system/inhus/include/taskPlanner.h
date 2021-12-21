#ifndef DEF_TASKPLANNER
#define DEF_TASKPLANNER

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "inhus/ComputePlan.h"
#include "inhus/Action.h"
#include <ros/package.h>
#include "task.h"
#include "geometry_msgs/Pose2D.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tinyxml.h>
#include <iostream>
#include "inhus/Goal.h"

using namespace std;

class TaskPlanner
{
public:
	TaskPlanner();
	bool computePlan(inhus::ComputePlan::Request& req, inhus::ComputePlan::Response& res);

	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	void readGoalsFromXML();
	void showGoals();

private:
	ros::NodeHandle nh_;
	ros::ServiceServer service_;

	string map_name_;
	string goal_file_name_;
	TiXmlDocument* doc_;
	vector<inhus::NamedGoal> named_goals_;

	ros::Publisher pub_log_;
};

#endif

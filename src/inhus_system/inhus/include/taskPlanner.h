#ifndef DEF_TASKPLANNER
#define DEF_TASKPLANNER

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "inhus/ComputePlan.h"
#include "inhus/Action.h"
#include "task.h"
#include "geometry_msgs/Pose2D.h"
#include <tf2/LinearMath/Quaternion.h>

class TaskPlanner
{
public:
	TaskPlanner();
	bool computePlan(inhus::ComputePlan::Request& req, inhus::ComputePlan::Response& res);

	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
	ros::ServiceServer service_;

	ros::Publisher pub_log_;
};

#endif

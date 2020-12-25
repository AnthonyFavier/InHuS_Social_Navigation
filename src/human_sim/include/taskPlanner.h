#ifndef DEF_TASKPLANNER
#define DEF_TASKPLANNER

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "human_sim/ComputePlan.h"
#include "geometry_msgs/Pose2D.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

class TaskPlanner
{
public:
	TaskPlanner();
	bool computePlan(human_sim::ComputePlan::Request& req, human_sim::ComputePlan::Response& res);

	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
	ros::ServiceServer service_;

	ros::Publisher pub_log_;
};

#endif

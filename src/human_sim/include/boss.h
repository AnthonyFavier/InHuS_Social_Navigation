#ifndef BOSS
#define BOSS

#include "ros/ros.h"
#include "human_sim/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int32.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <iostream>
#include "types.h"

using namespace std;

////////////////// AGENT MANAGER /////////////////
class AgentManager
{
public:
	AgentManager(ros::NodeHandle nh);
private:
	ros::NodeHandle nh_;
};
//////////////////////////////////////////////////

////////////////////// BOSS //////////////////////
class Boss
{
public:
	Boss();

	void showState();
	void askChoice();
	void cleanInput();

private:
	ros::NodeHandle nh_;
	vector<AgentManager> agent_managers_;

	int choice_;

	vector<GoalArea> list_goals_;
};
//////////////////////////////////////////////////

#endif

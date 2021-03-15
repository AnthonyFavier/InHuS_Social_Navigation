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

//////////////////////////////////////////////////

////////////////// AGENT MANAGER /////////////////
class AgentManager
{
public:
	AgentManager(string name, string topic_goal);
	virtual void publishGoal(GoalArea goal) = 0;
	string getName();

protected:
	ros::NodeHandle nh_;
	ros::Publisher pub_goal_;
	string name_;
	string topic_goal_;
};

class HumanManager : public AgentManager
{
public:
	HumanManager(string name, string topic_goal);
	void publishGoal(GoalArea goal);
private:
};

class RobotManager : public AgentManager
{
public:
	RobotManager(string name, string topic_goal);
	void publishGoal(GoalArea goal);
	geometry_msgs::PoseStamped getPose(human_sim::Goal goal);
private:
};
//////////////////////////////////////////////////

////////////////////// BOSS //////////////////////
class Boss
{
public:
	Boss();

	void appendAgent(AgentManager* agent);

	void showState();
	void askChoice();

	bool showAgents();

private:
	void cleanInput();

	void askSendGoal();
	void askScenario();
	void askSetAttitude();

	ros::NodeHandle nh_;
	vector<AgentManager*> agent_managers_;

	int choice_;

	vector<GoalArea> list_goals_;
};
//////////////////////////////////////////////////

#endif

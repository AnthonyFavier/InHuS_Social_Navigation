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

enum Type{HUMAN, ROBOT};

//////////////////////////////////////////////////

////////////////// AGENT MANAGER /////////////////
class AgentManager
{
public:
	AgentManager(string name);
	virtual void publishGoal(GoalArea goal) = 0;
	string getName();
	Type getType();
	bool isGoalDone();

protected:
	ros::NodeHandle nh_;
	ros::Publisher pub_goal_;
	string name_;
	Type type_;
	bool goal_done_;
};

class HumanManager : public AgentManager
{
public:
	HumanManager(string name);
	void publishGoal(GoalArea goal);
	void setAttitude(int attitude);
private:
	ros::Publisher pub_attitude_;

	ros::Subscriber sub_goal_done_;
	void goalDoneCB(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_goal_start_;
	void goalStartCB(const human_sim::Goal::ConstPtr& msg);

	bool goal_done_;
};

class RobotManager : public AgentManager
{
public:
	RobotManager(string name, string topic_goal, string topic_goal_status);
	void publishGoal(GoalArea goal);
	geometry_msgs::PoseStamped getPose(human_sim::Goal goal);

private:
	ros::Subscriber sub_goal_status_;
	void goalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
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
	bool showHumanAgents();

	void wait(float delay);

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

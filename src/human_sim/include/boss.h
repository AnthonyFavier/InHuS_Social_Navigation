#ifndef BOSS
#define BOSS

#include "ros/ros.h"
#include "human_sim/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
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
	virtual void showState() = 0;
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
	void publishManualCmd(geometry_msgs::Twist cmd);
	void showState();

	void setAttitude(int attitude);

private:
	ros::Publisher pub_attitude_;
	ros::Publisher pub_manual_cmd_;

	ros::Subscriber sub_goal_done_;
	void goalDoneCB(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_goal_start_;
	void goalStartCB(const human_sim::Goal::ConstPtr& msg);

	int current_attitude_;
};

class RobotManager : public AgentManager
{
public:
	RobotManager(string name, string topic_goal, string topic_goal_status);
	void publishGoal(GoalArea goal);
	void showState();

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
	void initGoals();

	void spawnThreadEndless();

	void appendAgent(AgentManager* agent);

	void showState();
	void askChoice();

	bool showAgents();
	bool showHumanAgents();

	void wait(float delay);

	void threadPubEndlessAgent1();
	void threadPubEndlessAgent2();

private:
	void cleanInput();

	void askSendGoal();
	void askScenario();
	void askSetAttitude();

	ros::NodeHandle nh_;
	vector<AgentManager*> agent_managers_;

	int choice_;
	bool endless_agent1_on_;
	int endless_agent1_;
	int endless_agent1_i_;
	bool endless_agent2_on_;
	int endless_agent2_;
	int endless_agent2_i_;

	vector<GoalArea> list_goals_;
	vector<GoalArea> endless_goals_agent1_;
	vector<GoalArea> endless_goals_agent2_;
};
//////////////////////////////////////////////////

#endif

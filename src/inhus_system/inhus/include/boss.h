#ifndef BOSS
#define BOSS

#include "ros/ros.h"
#include <ros/package.h>
#include "inhus/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Int32.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <tinyxml.h>
#include <iostream>
#include "types.h"

using namespace std;

enum Type{HUMAN, ROBOT};

struct Scenario
{
	string name;
	inhus::Goal init_h;
	inhus::Goal init_r;
	inhus::Goal goal_h;
	inhus::Goal goal_r;
};

//////////////////////////////////////////////////

////////////////// AGENT MANAGER /////////////////
class AgentManager
{
public:
	AgentManager(string name);
	virtual void publishGoal(inhus::Goal goal) = 0;
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
	void publishGoal(inhus::Goal goal);
	void publishManualCmd(geometry_msgs::Twist cmd);
	void showState();

	void setAttitude(int attitude);

private:
	ros::Publisher pub_attitude_;
	ros::Publisher pub_manual_cmd_;

	ros::Subscriber sub_goal_done_;
	void goalDoneCB(const inhus::Goal::ConstPtr& msg);
	ros::Subscriber sub_goal_start_;
	void goalStartCB(const inhus::Goal::ConstPtr& msg);

	int current_attitude_;
};

class RobotManager : public AgentManager
{
public:
	RobotManager(string name);
	void publishGoal(inhus::Goal goal);
	void showState();

	geometry_msgs::PoseStamped getPose(inhus::Goal goal);

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
	~Boss();

	void showGoals();

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
	void askEndlessMode();
	void askSetAttitude();

	bool showAskScenarios();
	bool showAskNamedGoal();
	void readGoalsFromXML();

	ros::NodeHandle nh_;
	vector<AgentManager*> agent_managers_;

	int choice_;
	bool endless_agent1_on_;
	int endless_agent1_;
	int endless_agent1_i_;
	bool endless_agent2_on_;
	int endless_agent2_;
	int endless_agent2_i_;
	ros::Duration endless_delay_;

	string map_name_;

	string goal_file_name_;
	TiXmlDocument* doc_;

	vector<inhus::Goal> pose_goals_;
	vector<inhus::Goal> named_goals_;
	vector<Scenario> scenarios_;
	vector<vector<inhus::Goal>> endless_goals_;
};
//////////////////////////////////////////////////

#endif

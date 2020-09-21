#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <string>
#include <time.h>
#include "human_sim/ComputePlan.h"

#include "task.h"
#include <actionlib/client/simple_action_client.h>

class Supervisor
{
public:
	Supervisor(ros::NodeHandle nh);

	void FSM();

	void findAGoal();
	void askPlan();
private:
	ros::NodeHandle nh_;

	enum StateGlobal{GET_GOAL, ASK_PLAN, EXEC_PLAN}; // wait plan ?
	StateGlobal state_global_;
	enum ChoiceGoalDecision{AUTONOMOUS, SPECIFIED};
	ChoiceGoalDecision choice_goal_decision_;

	bool goal_received_;
	Goal current_goal_;
	Plan plan_;

	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

	ros::Subscriber sub_human_pos_;
	void humanPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

	Pose2D human_pos_;

	actionlib::SimpleActionClient<human_sim::ActionHAction> client_action_;
	ros::ServiceClient client_plan_;
};

#endif

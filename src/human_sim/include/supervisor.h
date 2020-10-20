#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "human_sim/ComputePlan.h"
#include "human_sim/ChooseGoal.h"
#include "human_sim/Goal.h"
#include "std_msgs/Int32.h"

#include "task.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class Supervisor
{
public:
	Supervisor();

	void FSM();

	void findAGoal();
	void askPlan();
private:
	ros::NodeHandle nh_;

	enum StateGlobal{GET_GOAL, ASK_PLAN, EXEC_PLAN}; // wait plan ?
	StateGlobal state_global_;
	enum ChoiceGoalDecision{AUTONOMOUS=0, SPECIFIED};
	ChoiceGoalDecision choice_goal_decision_;

	bool goal_received_;
	human_sim::Goal current_goal_;
	Plan plan_;

	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_teleop_boss_;
	void teleopBossCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_operating_mode_;
	void operatingModeBossCallback(const std_msgs::Int32::ConstPtr& msg);
	ros::Subscriber sub_cancel_goal_;
	void cancelGoalCallback(const actionlib_msgs::GoalID::ConstPtr& msg);

	ros::Publisher pub_teleop_;
	ros::Publisher pub_goal_done_;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_action_;

	ros::ServiceClient client_plan_;
	ros::ServiceClient client_goal_;
};

#endif

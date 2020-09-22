#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <time.h>
#include "human_sim/ComputePlan.h"
#include "human_sim/Goal.h"
#include "std_msgs/Int32.h"

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
	enum ChoiceGoalDecision{AUTONOMOUS=0, SPECIFIED};
	ChoiceGoalDecision choice_goal_decision_;

	bool goal_received_;
	human_sim::Goal current_goal_;
	Plan plan_;

	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_teleoperation_boss_;
	void teleopBossCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_operating_mode_;
	void operatingModeBossCallback(const std_msgs::Int32::ConstPtr& msg);

	actionlib::SimpleActionClient<human_sim::HActionAction> client_action_;
	ros::ServiceClient client_plan_;
};

#endif

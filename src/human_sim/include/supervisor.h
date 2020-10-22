#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "human_sim/ComputePlan.h"
#include "human_sim/ChooseGoal.h"
#include "human_sim/Goal.h"
#include "human_sim/SetGetGoal.h"
#include "std_msgs/Int32.h"
#include "nav_msgs/Path.h"
#include <actionlib/client/simple_action_client.h>
#include "actionlib_msgs/GoalID.h"
#include <move_base_msgs/MoveBaseAction.h>

class Supervisor
{
public:
	Supervisor();

	void FSM();

	bool checkPlanFailure();
	void findAGoal();
	void askPlan();
private:
	ros::NodeHandle nh_;

	enum StateGlobal{GET_GOAL, ASK_PLAN, EXEC_PLAN, BLOCKED_BY_ROBOT}; // wait plan ?
	StateGlobal state_global_;
	enum ChoiceGoalDecision{AUTONOMOUS=0, SPECIFIED};
	ChoiceGoalDecision choice_goal_decision_;

	bool first_blocked_;

	bool goal_received_;
	human_sim::Goal current_goal_;
	Plan plan_;
	Pose2D human_pose_;

	bool reset_after_goal_aborted_;
	int goal_aborted_count_;
	float path_diff_threshold_;

	nav_msgs::Path current_path_;
	nav_msgs::Path previous_path_;
	bool first_path_received_;

	ros::Subscriber sub_human_pose_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_teleop_boss_;
	void teleopBossCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_operating_mode_;
	void operatingModeBossCallback(const std_msgs::Int32::ConstPtr& msg);
	ros::Subscriber sub_path_;
	void pathCallback(const nav_msgs::Path::ConstPtr& path);

	ros::Publisher pub_teleop_;
	ros::Publisher pub_goal_done_;
	ros::Publisher pub_cancel_goal_;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_action_;

	ros::ServiceClient client_plan_;
	ros::ServiceClient client_goal_;
	
	ros::ServiceServer service_set_get_goal_;
	bool setGetGoal(human_sim::SetGetGoal::Request &req, human_sim::SetGetGoal::Response &res);
};

#endif

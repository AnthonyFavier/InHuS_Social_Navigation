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
#include "human_sim/CancelGoalAndStop.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "visualization_msgs/Marker.h"

class Supervisor
{
public:
	Supervisor();

	void FSM();

	bool checkPlanFailure();
	void findAGoal();
	void askPlan();
	void updateMarkerPose(float x, float y, float alpha);

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

	int goal_aborted_count_;
	float path_diff_threshold_;
	ros::Duration dur_replan_;
	ros::Duration dur_replan_blocked_;
	ros::Time last_replan_;	
	int replan_success_nb_;

	nav_msgs::Path current_path_;
	nav_msgs::Path previous_path_;

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
	ros::Publisher pub_log_;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_action_;

	ros::ServiceClient client_plan_;
	ros::ServiceClient client_goal_;
	ros::ServiceClient client_make_plan_;
	ros::ServiceClient client_cancel_goal_and_stop_;
	
	ros::ServiceServer service_set_get_goal_;
	bool setGetGoal(human_sim::SetGetGoal::Request &req, human_sim::SetGetGoal::Response &res);

	visualization_msgs::Marker marker_rviz_;
	ros::Publisher pub_marker_rviz_;
};

#endif

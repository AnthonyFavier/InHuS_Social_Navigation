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

private:

////////// METHODS ////////// 

	bool checkPlanFailure();
	void askPlan();
	void updateMarkerPose(float x, float y, float alpha);
	void init();
	void initCheckFailure();
	float computePathLength(const nav_msgs::Path* path);

////////// ATTRIBUTES ////////// 

	// States //
	enum StateGlobal{GET_GOAL, ASK_PLAN, EXEC_PLAN, BLOCKED_BY_ROBOT}; // wait plan ?
	StateGlobal state_global_;
	enum BlockedState{NOT_FEASIBLE, ABORTED, LONGER};
	BlockedState blocked_state_;

	// Subscribers //
	ros::Subscriber sub_human_pose_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_path_;
	void pathCallback(const nav_msgs::Path::ConstPtr& path);

	// Publishers //
	ros::Publisher pub_goal_done_;
	ros::Publisher pub_marker_rviz_;
	ros::Publisher pub_log_;
	ros::Publisher pub_stop_cmd_;
	
	// Action client for move_base //
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client_action_;

	// Service clients //
	ros::ServiceClient client_plan_;
	ros::ServiceClient client_make_plan_;
	ros::ServiceClient client_cancel_goal_and_stop_;
	
	// Service servers //
	ros::ServiceServer service_set_get_goal_;
	bool setGetGoal(human_sim::SetGetGoal::Request &req, human_sim::SetGetGoal::Response &res);

	//// Variables ////
	ros::NodeHandle nh_;

	visualization_msgs::Marker marker_rviz_;

	std_msgs::String msg_; // to publish easily on log

	bool goal_received_;
	human_sim::Goal current_goal_;
	Plan plan_;
	Pose2D human_pose_;

	bool first_blocked_;
	bool first_not_feasible_;

	int goal_aborted_count_;
	ros::Rate replan_freq_;
	ros::Rate blocked_ask_path_freq_;
	ros::Rate not_feasible_check_pose_freq_;
	int blocked_nb_ask_success_unblock_;
	ros::Time last_replan_;	
	int replan_success_nb_;
	float absolute_path_length_diff_;
	float ratio_path_length_diff_;
	float not_feasible_dist_threshold_unblock_;
	float not_feasible_theta_threshold_unblock_;
	int not_feasible_nb_same_pose_block_;
	float replan_dist_stop_;

	int same_human_pose_count_;
	Pose2D last_human_pose_;
	ros::Time last_check_human_pose_;

	nav_msgs::Path current_path_;
	nav_msgs::Path previous_path_;
};

#endif

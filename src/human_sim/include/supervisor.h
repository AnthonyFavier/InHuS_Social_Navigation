#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "human_sim/ComputePlan.h"
#include "human_sim/Goal.h"
#include "human_sim/Signal.h"
#include "human_sim/ActionBool.h"
#include "move_human/PlaceRobot.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/Marker.h"

class Supervisor
{
public:
	Supervisor();

	void FSM();

private:

////////// METHODS //////////

	bool checkBlocked();
	void askPlan();
	void updateMarkerPose(float x, float y, float alpha);
	void init();
	void initCheckBlocked(){};
	float computePathLength(const nav_msgs::Path* path);
	int cutPath(const nav_msgs::Path& path1, nav_msgs::Path& path2);

////////// ATTRIBUTES //////////

	// States //
	enum StateGlobal{GET_GOAL, ASK_PLAN, EXEC_PLAN, APPROACH, BLOCKED, SUSPENDED};
	StateGlobal global_state_;
	enum BlockedState{NOT_FEASIBLE, ABORTED, LONGER};
	BlockedState blocked_state_;
	enum ApproachState{CHECKING, REPLANNING};
	ApproachState approach_state_;

	// Subscribers //
	ros::Subscriber sub_human_pose_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pose_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_path_;
	void pathCallback(const nav_msgs::Path::ConstPtr& path);
	ros::Subscriber sub_status_move_base_;
	void stateMoveBaseCB(const actionlib_msgs::GoalStatusArray::ConstPtr& status);

	// Publishers //
	ros::Publisher pub_cancel_goal_;
	ros::Publisher pub_goal_move_base_;
	ros::Publisher pub_goal_done_;
	ros::Publisher pub_marker_rviz_;
	ros::Publisher pub_log_;
	ros::Publisher pub_stop_cmd_;

	// Service clients //
	ros::ServiceClient client_plan_;
	ros::ServiceClient client_make_plan_;
	ros::ServiceClient client_cancel_goal_and_stop_;
	ros::ServiceClient client_place_robot_hm_;
	ros::ServiceClient client_check_conflict_;
	ros::ServiceClient client_init_check_conflict_;

	// Services
	move_human::PlaceRobot srv_place_robot_hm_;
	nav_msgs::GetPlan srv_get_plan_;
	human_sim::Signal srv_cancel_stop_;

	// Service servers //
	ros::ServiceServer service_set_get_goal_;
	bool setGetGoal(human_sim::Signal::Request &req, human_sim::Signal::Response &res);
	ros::ServiceServer service_suspend_;
	bool srvSuspend(human_sim::Signal::Request &req, human_sim::Signal::Response &res);

	//// Variables ////
	ros::NodeHandle nh_;

	visualization_msgs::Marker marker_rviz_;

	std_msgs::String msg_; // to publish easily on log

	actionlib_msgs::GoalStatus goal_status_;

	bool goal_received_;
	human_sim::Goal current_goal_;
	Plan plan_;
	geometry_msgs::Pose2D human_pose_;
	geometry_msgs::Pose2D robot_pose_;

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
	float approach_dist_;
	ros::Rate approach_freq_;
	ros::Duration place_robot_delay_;

	int same_human_pose_count_;
	geometry_msgs::Pose2D last_human_pose_;
	ros::Time last_check_human_pose_;

	nav_msgs::Path current_path_;
};

#endif

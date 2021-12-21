#ifndef DEF_SUPERVISOR
#define DEF_SUPERVISOR

#include "ros/ros.h"
#include <string>
#include <vector>
#include <math.h>
#include "task.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "inhus/ComputePlan.h"
#include "inhus/Goal.h"
#include "std_srvs/Empty.h"
#include "inhus/ActionBool.h"
#include "inhus_navigation/PlaceRobot.h"
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
#include "manipulate_path.hpp"
#include "types.h"

class Supervisor
{
public:
	Supervisor();

	void FSM();

private:

////////// METHODS //////////

	void askPlan();
	void updateMarkerPose(float x, float y, float alpha);
	void statePrint();
	move_base_msgs::MoveBaseGoal getMoveBaseGoal(geometry_msgs::Pose2D pose);


////////// ATTRIBUTES //////////

	// Subscribers //
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const inhus::Goal::ConstPtr& msg);
	ros::Subscriber sub_path_; nav_msgs::Path current_path_;
	void pathCallback(const nav_msgs::Path::ConstPtr& path);
	ros::Subscriber sub_status_move_base_; actionlib_msgs::GoalStatus goal_status_;
	void stateMoveBaseCB(const actionlib_msgs::GoalStatusArray::ConstPtr& status);

	// Publishers //
	ros::Publisher pub_goal_move_base_;
	ros::Publisher pub_goal_done_;
	ros::Publisher pub_marker_rviz_;
	ros::Publisher pub_log_; std_msgs::String msg_; // to publish easily on log

	// Service clients //
	ros::ServiceClient client_plan_;
	ros::ServiceClient client_place_robot_hm_;
	ros::ServiceClient client_check_conflict_;
	ros::ServiceClient client_init_check_conflict_;
	ros::ServiceClient client_init_first_path_conflict_;

	// Services
	inhus_navigation::PlaceRobot srv_place_robot_hm_;

	// Service servers //
	ros::ServiceServer server_set_wait_goal_;
	bool srvSetWaitGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	ros::ServiceServer server_suspend_;
	bool srvSuspend(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	ros::ServiceServer server_resume_;
	bool srvResume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	//// Variables ////
	ros::NodeHandle nh_;

	// States //
	enum StateGlobal{WAIT_GOAL, ASK_PLAN, EXEC_PLAN, SUSPENDED};
	StateGlobal global_state_;
	StateGlobal global_state_previous_;

	// Params
	bool replan_active_;
	ros::Rate replan_freq_;
	ros::Rate check_freq_;
	float replan_dist_stop_;
	ros::Duration place_robot_delay_;

	// Other
	inhus::Goal current_goal_;
	Plan plan_;
	bool goal_received_;
	inhus::Action current_action_;

	ros::Time start_time_wait_action_;
	ros::Duration dur_wait_action_;

	visualization_msgs::Marker marker_rviz_;
	ros::Time last_replan_;
	ros::Time last_check_;
};

#endif

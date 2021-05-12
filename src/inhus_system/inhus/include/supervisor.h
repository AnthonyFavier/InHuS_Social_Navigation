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
#include "inhus/Signal.h"
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

////////// ATTRIBUTES //////////

	// Subscribers //
	ros::Subscriber sub_human_pose_; geometry_msgs::Pose2D human_pose_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pose_; geometry_msgs::Pose2D robot_pose_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
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
	ros::Publisher pub_stop_cmd_;

	// Service clients //
	ros::ServiceClient client_plan_;
	ros::ServiceClient client_place_robot_hm_;
	ros::ServiceClient client_check_conflict_;
	ros::ServiceClient client_init_check_conflict_;

	// Services
	inhus_navigation::PlaceRobot srv_place_robot_hm_;

	// Service servers //
	ros::ServiceServer server_set_wait_goal_;
	bool srvSetWaitGoal(inhus::Signal::Request &req, inhus::Signal::Response &res);
	ros::ServiceServer server_suspend_;
	bool srvSuspend(inhus::Signal::Request &req, inhus::Signal::Response &res);
	ros::ServiceServer server_resume_;
	bool srvResume(inhus::Signal::Request &req, inhus::Signal::Response &res);

	//// Variables ////
	ros::NodeHandle nh_;

	// States //
	enum StateGlobal{WAIT_GOAL, ASK_PLAN, EXEC_PLAN, SUSPENDED};
	StateGlobal global_state_;
	StateGlobal global_state_previous_;

	// Params
	ros::Rate replan_freq_;
	float replan_dist_stop_;
	ros::Duration place_robot_delay_;

	// Other
	inhus::Goal current_goal_;
	Plan plan_;
	bool goal_received_;

	visualization_msgs::Marker marker_rviz_;
	ros::Time last_replan_;
};

#endif

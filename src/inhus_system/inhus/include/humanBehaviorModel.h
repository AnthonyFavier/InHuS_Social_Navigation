#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include <ros/package.h>
#include <tinyxml.h>
#include <vector>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <math.h>
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "navfn/MakeNavPlan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include "types.h"
#include "manipulate_path.hpp"
#include "inhus/PoseVel.h"
#include "inhus/Goal.h"
#include "inhus/ActionBool.h"
#include "inhus_navigation/PlaceRobot.h"

class ConflictManager
{
public:
	ConflictManager(ros::NodeHandle nh, bool* want_robot_placed);

	void updateData(inhus::PoseVel h_pose_vel, inhus::PoseVel r_pose_vel);
	bool checkConflict();
	void loop();
	bool inConflict(){return state_global_!=IDLE;};
	bool inApproach(){return state_global_==APPROACH;};

private:
	ros::NodeHandle nh_;

	// Params
	float absolute_path_length_diff_;
	float ratio_path_length_diff_;
	ros::Rate approach_freq_;
	float replan_dist_stop_;
	float approach_dist_;
	ros::Rate blocked_ask_path_freq_;
	ros::Rate replan_freq_;

	// Service servers
	ros::ServiceServer server_check_conflict_;
	bool srvCheckConflict(inhus::ActionBool::Request &req, inhus::ActionBool::Response &res);
	ros::ServiceServer server_init_conflict_;
	bool srvInitCheckConflict(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	ros::ServiceServer server_init_first_path_;
	bool srvInitFirstPath(inhus::ActionBool::Request &req, inhus::ActionBool::Response &res);


	// Service clients
	ros::ServiceClient client_cancel_goal_and_stop_;
	ros::ServiceClient client_make_plan_;
	ros::ServiceClient client_update_robot_map_;
	ros::ServiceClient client_resume_supervisor_;

	// srv
	inhus_navigation::PlaceRobot srv_place_robot_hm_;
    navfn::MakeNavPlan srv_make_plan_;
	std_srvs::Empty srv_signal_;

	// Publishers
	ros::Publisher pub_log_; std_msgs::String msg_;
	ros::Publisher pub_cancel_goal_;
	ros::Publisher pub_goal_move_base_;
	ros::Publisher pub_vel_cmd_;
	ros::Publisher pub_perturbed_cmd_;

	// Other
	enum StateGlobal{IDLE, APPROACH, BLOCKED};
	StateGlobal state_global_;
	enum StateApproach{FIRST, CHECKING, PLACE_ROBOT, REPLANNING, REMOVE_ROBOT};
	StateApproach state_approach_;
	enum StateBlocked{NO_PATH, LONGER};
	StateBlocked state_blocked_;

	static const int nb_last_vels_ = 4;
	geometry_msgs::Twist last_vels_[nb_last_vels_];
	geometry_msgs::Twist mean_vel_;

	actionlib_msgs::GoalStatus goal_status_;
	ros::Time last_replan_;

	ros::Duration delay_place_robot_;

	inhus::PoseVel h_pose_vel_;
	inhus::PoseVel r_pose_vel_;

	nav_msgs::Path current_path_;
	nav_msgs::Path previous_path_;

	move_base_msgs::MoveBaseGoal current_action_;

	bool* want_robot_placed_;
};

///////////////////////////////////////////////////////////

class HumanBehaviorModel
{
public:
	HumanBehaviorModel(ros::NodeHandle nh);

	void processSimData();
	void publishModelData();
	void attitudes();
	void pubDist();
	void computeTTC();
	void computeRelSpd();
	void computeSurprise();
	void testSeeRobot();
	void updateConflictManager();
	void initConflictManager(ConflictManager* conflict_manager);
	void conflictManagerLoop();
	void updatePoseMarkers();

	bool initDone();

	bool want_robot_placed_; // enable conflictManager to write it

private:

////////// METHODS //////////

	inhus::Goal chooseGoal(bool random);
	void showGoals();
	void readGoalsFromXML();
	void attNonStop();
	void attRandom();
	void attStopLook();
	void attHarass();
	void publishGoal(inhus::Goal goal);
	bool testObstacleView(geometry_msgs::Pose2D A_real, geometry_msgs::Pose2D B_real);
	bool testFOV(geometry_msgs::Pose2D A, geometry_msgs::Pose2D B, float fov);
	void updateRobotOnMap();

////////// ATTRIBUTES //////////

	// Attitudes //
	enum Attitude{NONE=0, NON_STOP, RANDOM, STOP_LOOK, HARASS};
	Attitude attitude_;

	// Sub state attitudes //
	enum SubAttitudeStopLook{WAIT_ROBOT, STOP, LOOK_AT_ROBOT, RESUME_GOAL, OVER};
	SubAttitudeStopLook sub_stop_look_;
	enum SubAttitudeHarass{INIT, HARASSING};
	SubAttitudeHarass sub_harass_;

	// Subscribers //
	ros::Subscriber sub_h_pose_vel_;
	void hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);
	ros::Subscriber sub_r_pose_vel_;
	void rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);
	ros::Subscriber sub_cmd_geo_;
	void cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_goal_done_;
	void goalDoneCallback(const inhus::Goal::ConstPtr& msg);
	ros::Subscriber sub_set_attitude_;
	void setAttitudeCallback(const std_msgs::Int32::ConstPtr& msg);
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const inhus::Goal::ConstPtr& goal);
	ros::Subscriber sub_pov_map_;
	void povMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);

	// Publishers //
	ros::Publisher pub_new_goal_;
	ros::Publisher pub_h_pose_vel_;
	ros::Publisher pub_r_pose_vel_;
	ros::Publisher pub_perturbed_cmd_;
	ros::Publisher pub_cancel_goal_;
	ros::Publisher pub_goal_move_base_;
	ros::Publisher pub_log_;
	ros::Publisher pub_pose_marker_human_;
	ros::Publisher pub_pose_marker_robot_;

	// Service Clients //
	ros::ServiceClient client_set_wait_goal_;
	ros::ServiceClient client_cancel_goal_and_stop_;
	ros::ServiceClient client_place_robot_;
	ros::ServiceClient client_suspend_supervisor_;
	ros::ServiceClient client_resume_supervisor_;

	// Service Server //
	ros::ServiceServer server_place_robot_;
	bool srvPlaceRobotHM(inhus_navigation::PlaceRobot::Request& req, inhus_navigation::PlaceRobot::Response& res);
	ros::ServiceServer server_update_robot_map_;
	bool srvUpdateRobotMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

	// Services //
	inhus_navigation::PlaceRobot srv_place_robot_;
	std_srvs::Empty srv_signal_;

	//// Variables ////
	ros::NodeHandle nh_;

	inhus::PoseVel sim_h_pose_vel_;
	inhus::PoseVel sim_r_pose_vel_;

	inhus::PoseVel model_h_pose_vel_;
	inhus::PoseVel model_r_pose_vel_;

	double ttc_;
	double relative_speed_;
	float human_radius_;
	float robot_radius_;
	float dist_radius_inflation_;
	float radius_sum_sq_;
	float dist_;

	visualization_msgs::MarkerArray human_pose_marker_;
	visualization_msgs::MarkerArray robot_pose_marker_;

	inhus::Goal current_goal_;
	inhus::Goal previous_goal_;

	std::string map_name_;
	std::string goal_file_name_;
	TiXmlDocument* doc_;
	std::vector<inhus::Goal> known_goals_;

	bool executing_plan_;

	std_msgs::String msg_log_;

	bool hcb_;
	bool rcb_;

	// POV //
	bool know_robot_pose_;
	bool pmcb_;
	std::vector<std::vector<int>> g_map_;
	float fov_;
	float resol_pov_map_;
	ros::Rate check_see_robot_freq_;
	ros::Time last_check_see_robot_;
	float offset_pov_map_x_;
	float offset_pov_map_y_;
	ros::Time last_seen_robot_;
	ros::Duration delay_forget_robot_;
	bool see_;

	// Surprise
	ros::Duration surprise_full_increase_durr_;
	ros::Duration surprise_full_decrease_durr_;
	float surprise_dist_;
	ros::Time surprise_last_compute_;
	float surprise_seen_ratio_;

	// ratio perturbation
	float ratio_perturbation_cmd_;

	// chance to find a new goal
	ros::Rate b_random_try_freq_;
	int b_random_chance_choose_;
	ros::Time last_time_;

	// Near robot distance
	float b_stop_look_dist_near_robot_;
	ros::Time time_stopped_;
	ros::Duration b_stop_look_stop_dur_;

	// Harass
	float b_harass_dist_in_front_;
	ros::Rate b_harass_replan_freq_;
	ros::Time last_harass_;

	// Conflict Manager
	ConflictManager* conflict_manager_;
};

#endif

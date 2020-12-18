#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include <vector>
#include <time.h>
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/Goal.h"
#include "human_sim/SetGetGoal.h"
#include "human_sim/CancelGoalAndStop.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <tf2/LinearMath/Quaternion.h>

#define PI 3.1415926535897932384626433832795

struct GoalArea
{
	human_sim::Goal goal;
	float radius;
};

class HumanModel
{
public:
	HumanModel();

	void processSimData();
	void publishModelData();
	void behaviors();
	void pubDist();


private:

////////// METHODS ////////// 

	human_sim::Goal chooseGoal(bool random);
	void nonStop();
	void newRandomGoalGeneration();
	void stopLookRobot();
	void harassRobot();
	void publishGoal(human_sim::Goal& goal);

////////// ATTRIBUTES ////////// 

	// Behaviors //
	enum Behavior{NONE=0, NON_STOP, RANDOM, STOP_LOOK, HARASS};
	Behavior behavior_;

	// Sub state behaviors //
	enum SubBehaviorStopLook{WAIT_ROBOT, STOP, LOOK_AT_ROBOT, RESUME_GOAL, OVER};
	SubBehaviorStopLook sub_stop_look_;
	enum SubBehaviorHarass{INIT, HARASSING};
	SubBehaviorHarass sub_harass_;

	// Subscribers //
	ros::Subscriber sub_pose_;
	void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_vel_;
	void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_robot_pose_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_vel_;
	void robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_cmd_geo_;
	void cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_goal_done_;
	void goalDoneCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_set_behavior_;
	void setBehaviorCallback(const std_msgs::Int32::ConstPtr& msg);
	ros::Subscriber sub_new_goal_;
	void newGoalCallback(const human_sim::Goal::ConstPtr& goal);
	ros::Subscriber sub_stop_cmd_;
	void stopCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd);

	// Publishers //
	ros::Publisher pub_new_goal_;
	ros::Publisher pub_human_pose_;
	ros::Publisher pub_human_vel_;
	ros::Publisher pub_robot_pose_;
	ros::Publisher pub_robot_vel_;
	ros::Publisher pub_perturbed_cmd_;
	ros::Publisher pub_cancel_goal_;
	ros::Publisher pub_goal_move_base_;
	ros::Publisher pub_log_;

	// Service Clients //
	ros::ServiceClient client_set_get_goal_;
	ros::ServiceClient client_cancel_goal_and_stop_;

	//// Variables ////
	ros::NodeHandle nh_;

	geometry_msgs::Pose2D sim_pose_;
	geometry_msgs::Twist sim_vel_;
	geometry_msgs::Pose2D sim_robot_pose_;
	geometry_msgs::Twist sim_robot_vel_;

	geometry_msgs::Pose2D model_pose_;
	geometry_msgs::Twist model_vel_;
	geometry_msgs::Pose2D model_robot_pose_;
	geometry_msgs::Twist model_robot_vel_;

	human_sim::Goal current_goal_;
	human_sim::Goal previous_goal_;

	std::vector<GoalArea> known_goals_;

	bool executing_plan_;

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
};

#endif

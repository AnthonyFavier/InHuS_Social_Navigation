#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/Goal.h"
#include <vector>
#include "human_sim/ChooseGoal.h"
#include <time.h>
#include <math.h>
#include "actionlib_msgs/GoalID.h"
#include "std_msgs/Int32.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "human_sim/SetGetGoal.h"
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
	bool chooseGoalSrv(human_sim::ChooseGoal::Request& req, human_sim::ChooseGoal::Response& res);
	human_sim::Goal chooseGoal();
	void newRandomGoalGeneration(bool toss);
	void stopLookRobot();
	void harassRobot();
	void behaviors();

private:
	ros::NodeHandle nh_;

	enum Behavior{NONE=0, RANDOM, STOP_LOOK, HARASS};
	enum SubBehaviorStopLook{WAIT_ROBOT, STOP, LOOK_AT_ROBOT, NEW_GOAL, OVER};
	enum SubBehaviorHarass{INIT, HARASSING};
	Behavior behavior_;
	SubBehaviorStopLook sub_stop_look_;
	SubBehaviorHarass sub_harass_;

	Pose2D sim_pose_;
	Pose2D sim_robot_pose_;

	Pose2D model_pose_;
	Pose2D model_robot_pose_;

	ros::Subscriber sub_pose_;
	void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pose_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_cmd_geo_;
	void cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg);
	ros::Subscriber sub_goal_done_;
	void goalDoneCallback(const human_sim::Goal::ConstPtr& msg);
	ros::Subscriber sub_set_behavior_;
	void setBehaviorCallback(const std_msgs::Int32::ConstPtr& msg);

	ros::Publisher pub_new_goal_;
	ros::Publisher pub_op_mode_;
	ros::Publisher pub_human_pose_;
	ros::Publisher pub_robot_pose_;
	ros::Publisher pub_perturbated_cmd_;
	ros::Publisher pub_cancel_goal_;
	ros::Publisher pub_goal_move_base_;

	ros::ServiceServer service_;

	ros::ServiceClient client_set_get_goal_;

	float ratio_perturbation_;

	human_sim::Goal current_goal_;
	human_sim::Goal previous_goal_;

	std::vector<GoalArea> known_goals_;

	ros::Time last_time_;
	ros::Duration delay_think_about_new_goal_;
	int chance_decide_new_goal_;

	float dist_near_robot_;

	float dist_in_front_;
	ros::Duration delay_harass_replan_;
	ros::Time last_harass_;
};

#endif

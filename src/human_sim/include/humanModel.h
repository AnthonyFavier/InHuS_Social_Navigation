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

#define PI 3.1415926535897932384626433832795

struct GoalArea
{
	human_sim::Goal goal;
	float radius;
};

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);

	void processSimData();
	void publishModelData();
	bool chooseGoalSrv(human_sim::ChooseGoal::Request& req, human_sim::ChooseGoal::Response& res);
	human_sim::Goal chooseGoal();
	void newRandomGoalGeneration(bool toss);
	void stopNearRobot();
	void behaviors();

private:
	ros::NodeHandle nh_;

	enum Behavior{NONE, RANDOM, STOP_NEAR};
	enum SubBehaviorStopNear{WAIT_ROBOT, STOP, WAIT_AFTER, NEW_GOAL, OVER};
	Behavior behavior_;
	SubBehaviorStopNear sub_stop_near_;

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

	ros::Publisher pub_new_goal_;
	ros::Publisher pub_human_pose_;
	ros::Publisher pub_robot_pose_;
	ros::Publisher pub_perturbated_cmd_;
	ros::Publisher pub_cancel_goal_;

	ros::ServiceServer service_;

	float ratio_perturbation_;

	human_sim::Goal current_goal_;
	human_sim::Goal previous_goal_;

	std::vector<GoalArea> known_goals_;

	ros::Time last_time_;
	ros::Duration delay_think_about_new_goal_;
	int chance_decide_new_goal_;

	float dist_near_robot_;

};

#endif

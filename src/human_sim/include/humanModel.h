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

#define PI 3.1415926535897932384626433832795

class Map
{
public:
	Map(float rx, float ry, float tile_size);
	enum Tile{FREE=0, OBSTACLE, GOAL};
	void show();

	std::vector<std::vector<Tile>> map_;

	int getNX(){return map_.size();};
	int getNY(){if(map_.size()){return map_[0].size();}else{return 0;}};
	float getRSizeX(){return real_size_x_;};
	float getRSizeY(){return real_size_y_;};
	float getTileSize(){return tile_size_;};
private:
	float real_size_x_;
	float real_size_y_;
	float tile_size_;
};

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);

	void processSimData();
	void publishModelData();
	bool chooseGoalSrv(human_sim::ChooseGoal::Request& req, human_sim::ChooseGoal::Response& res);
	human_sim::Goal chooseGoal();
	void newGoalGeneration();

private:
	ros::NodeHandle nh_;

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

	ros::ServiceServer service_;

	float ratio_perturbation_;

	human_sim::Goal current_goal_;
	human_sim::Goal previous_goal_;

	//Map map_;
	std::vector<human_sim::Goal> known_goals_;

	ros::Time last_time_;
	ros::Duration delay_think_about_new_goal_;
	int chance_decide_new_goal_;
};

#endif

#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/Goal.h"
#include <vector>

class Map
{
public:
	Map(float rx, float ry, float tile_size);
	enum Tile{FREE=0, OBSTACLE, GOAL};
	void show();
private:
	float real_size_x_;
	float real_size_y_;
	float tile_size_;
	std::vector<std::vector<Tile>> map_;
};

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);

	void processSimData();
	void publishModelData();

private:
	ros::NodeHandle nh_;

	Pose2D sim_pose_;
	Pose2D sim_robot_pose_;

	Pose2D model_pose_;
	Pose2D model_robot_pose_;

	geometry_msgs::Twist last_cmd_geo_;

	ros::Subscriber sub_pose_;
	void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pose_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_cmd_geo_;
	void cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg);

	ros::Publisher pub_new_goal_;
	ros::Publisher pub_human_pose_;
	ros::Publisher pub_robot_pose_;
	ros::Publisher pub_noisy_cmd_;

	Map map_;
};

#endif

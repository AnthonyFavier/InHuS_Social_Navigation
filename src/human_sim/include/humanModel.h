#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include "type.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/Goal.h"

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);

	void processSimData();
	void publishModelData();

private:
	ros::NodeHandle nh_;

	Pose2D sim_pos_;
	Pose2D sim_robot_pos_;

	Pose2D model_pos_;
	Pose2D model_robot_pos_;

	geometry_msgs::Twist last_cmd_geo_;

	ros::Subscriber sub_pos_;
	void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pos_;
	void robotPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_cmd_geo_;
	void cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg);

	ros::Publisher pub_new_goal_;
	ros::Publisher pub_human_pos_;
	ros::Publisher pub_robot_pos_;
	ros::Publisher pub_noisy_cmd_;

	//Map map;
};

#endif

#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"
#include "type.h"
#include "geometry_msgs/Pose2D.h"

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);
private:
	ros::NodeHandle nh_;

	Pose2D pos_;
	Pose2D robot_pos_;

	ros::Subscriber sub_pos_;
	void posCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	ros::Subscriber sub_robot_pos_;
	void robotPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

	//Map map;
};

#endif

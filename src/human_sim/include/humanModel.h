#ifndef DEF_HUMAN_MODEL
#define DEF_HUMAN_MODEL

#include "ros/ros.h"

class HumanModel
{
public:
	HumanModel(ros::NodeHandle nh);
private:
	ros::NodeHandle nh_;
};

#endif

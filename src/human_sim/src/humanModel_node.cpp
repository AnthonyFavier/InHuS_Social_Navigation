#include "humanModel.h"

/////////////////////// HUMAN MODEL ///////////////////////

HumanModel::HumanModel(ros::NodeHandle nh)
{
	nh_ = nh;
	sub_pos_ = nh_.subscribe("human_pos", 100, &HumanModel::posCallback, this);
	sub_robot_pos_ = nh_.subscribe("robot_pos", 100, &HumanModel::robotPosCallback, this);
}

void HumanModel::posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	pos_.x=msg->x;
	pos_.y=msg->y;
	pos_.theta=msg->theta;
}

void HumanModel::robotPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	robot_pos_.x=msg->x;
	robot_pos_.y=msg->y;
	robot_pos_.theta=msg->theta;
}

////////////////////////// MAIN ///////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	ros::NodeHandle nh;

	HumanModel human_model(nh);

	ros::spin();
}

///////////////////////////////////////////////////////////

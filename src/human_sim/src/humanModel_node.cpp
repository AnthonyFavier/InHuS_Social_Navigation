#include "humanModel.h"

/////////////////////// HUMAN MODEL ///////////////////////

HumanModel::HumanModel(ros::NodeHandle nh)
{
	nh_ = nh;

	Pose2D zero;
	zero.x = 	0;
	zero.y = 	0;
	zero.theta = 	0;
	sim_pos_ = 	   zero;
	sim_robot_pos_=    zero;
	model_pos_ = 	   zero;
	model_robot_pos_ = zero;

	sub_pos_ = 	 nh_.subscribe("sim/human_pos", 100, &HumanModel::posCallback, this);
	sub_robot_pos_ = nh_.subscribe("sim/robot_pos", 100, &HumanModel::robotPosCallback, this);

	pub_new_goal_ = 	nh_.advertise<human_sim::Goal>("boss/new_goal", 100);
	pub_human_pos_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/human_pos", 100);
	pub_robot_pos_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/robot_pos", 100);

	printf("I am human\n");
}

void HumanModel::posCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_pos_.x=msg->x;
	sim_pos_.y=msg->y;
	sim_pos_.theta=msg->theta;
}

void HumanModel::robotPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_robot_pos_.x=msg->x;
	sim_robot_pos_.y=msg->y;
	sim_robot_pos_.theta=msg->theta;
}

void HumanModel::processSimData()
{
	model_pos_ = sim_pos_;
	model_robot_pos_ = sim_robot_pos_;
}

void HumanModel::publishModelData()
{
	geometry_msgs::Pose2D pos;
	pos.x = model_pos_.x;
	pos.y = model_pos_.y;
	pos.theta = model_pos_.theta;
	pub_human_pos_.publish(pos);

	pos.x = model_robot_pos_.x;
	pos.y = model_robot_pos_.y;
	pos.theta = model_robot_pos_.theta;
	pub_robot_pos_.publish(pos);
}

////////////////////////// MAIN ///////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	ros::NodeHandle nh;

	HumanModel human_model(nh);

	ros::Rate rate(5);

	while(ros::ok())
	{
		// process data from simu
		// -> memory etc..
		human_model.processSimData();

		// Publish data as perceived by the human model
		human_model.publishModelData();

		rate.sleep();
		ros::spinOnce();
	}
}

///////////////////////////////////////////////////////////

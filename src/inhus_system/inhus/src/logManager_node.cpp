#include "logManager.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("inhus");

	log_file_inhus_.open(path_ + "/logs/inhus_logs/log.txt");
	log_file_inhus_ << "LOG STARTS : " << ros::Time::now() << endl;
	sub_log_ = nh_.subscribe("log", 100, &LogManager::logCallback, this);
	sub_h_pose_vel_ = nh_.subscribe("known/human_pose_vel", 100, &LogManager::hPoseVelCallback, this);
	sub_r_pose_vel_ = nh_.subscribe("known/robot_pose_vel", 100, &LogManager::rPoseVelCallback, this);

	log_file_inhus_poses_.open(path_ + "/logs/inhus_logs/poseLog.txt");
	log_file_inhus_poses_ << "LOG STARTS : " << ros::Time::now() << endl;
}

LogManager::~LogManager()
{
	log_file_inhus_.close();
	log_file_inhus_poses_.close();
}

////////////////////// LOG_FILE_INHUS ///////////////////////
void LogManager::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_inhus_ << ros::Time::now() << " : " <<  msg->data << endl;
}

void LogManager::hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	if(ros::Time::now().toSec() != 0.0)
		log_file_inhus_poses_ << ros::Time::now() << " : H " << msg->pose.x << " " << msg->pose.y  << " " << msg->pose.theta << endl;
		log_file_inhus_ << ros::Time::now() << " : LOG VEL_H " << std::to_string(sqrt(pow(msg->vel.linear.x,2) + pow(msg->vel.linear.y,2))) << endl;
}

void LogManager::rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	if(ros::Time::now().toSec() != 0.0)
		log_file_inhus_poses_ << ros::Time::now() << " : R " << msg->pose.x << " " << msg->pose.y  << " " << msg->pose.theta << endl;
		log_file_inhus_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(msg->vel.linear.x,2) + pow(msg->vel.linear.y,2))) << endl;
}

//////////////////////// MAIN ///////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "logManager");

	LogManager log_manager;

	ros::spin();

	return 0;
}

/////////////////////////////////////////////////////////////

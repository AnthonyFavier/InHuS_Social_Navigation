#include "logManager.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("inhus");

	log_file_inhus_.open(path_ + "/logs/inhus_logs/log.txt");
	log_file_inhus_ << "LOG STARTS : " << ros::Time::now() << endl;
	sub_log_ = nh_.subscribe("log", 100, &LogManager::logCallback, this);
	sub_vel_H_ = nh_.subscribe("known/human_vel", 100, &LogManager::velHCallback, this);
	sub_vel_R_ = nh_.subscribe("known/robot_vel", 100, &LogManager::velRCallback, this);

	log_file_inhus_poses_.open(path_ + "/logs/inhus_logs/poseLog.txt");
	log_file_inhus_poses_ << "LOG STARTS : " << ros::Time::now() << endl;
	sub_pose_H_ = nh_.subscribe("interface/in/human_pose", 100, &LogManager::poseHCallback, this);
	sub_pose_R_ = nh_.subscribe("interface/in/robot_pose", 100, &LogManager::poseRCallback, this);
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

void LogManager::velHCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

	log_file_inhus_ << ros::Time::now() << " : LOG VEL_H " << std::to_string(sqrt(pow(msg->linear.x,2) + pow(msg->linear.y,2))) << endl;
}

void LogManager::velRCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	log_file_inhus_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(msg->linear.x,2) + pow(msg->linear.y,2))) << endl;
}

///////////////// LOG_FILE_INHUS_POSES /////////////////////
void LogManager::poseHCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	log_file_inhus_poses_ << ros::Time::now() << " : H " << std::to_string(ros::Time::now().toSec()) << " " << msg->x << " " << msg->y << endl;
}

void LogManager::poseRCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	log_file_inhus_poses_ << ros::Time::now() << " : R " << std::to_string(ros::Time::now().toSec()) << " " << msg->x << " " << msg->y << endl;
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

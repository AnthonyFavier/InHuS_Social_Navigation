#include "logManager.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("human_sim");
	log_file_.open(path_ + "/logs/log.txt");
	log_file_ << "LOG STARTS : " << ros::Time::now() << endl;

	sub_log_ = nh_.subscribe("log", 100, &LogManager::logCallback, this);
	sub_vel_H_ = nh_.subscribe("known/human_vel", 100, &LogManager::velHCallback, this);
	sub_vel_R_ = nh_.subscribe("known/robot_vel", 100, &LogManager::velRCallback, this);
}

LogManager::~LogManager()
{
	log_file_.close();
}

void LogManager::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : " <<  msg->data << endl;
}

void LogManager::velHCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

	log_file_ << ros::Time::now() << " : LOG VEL_H " << std::to_string(sqrt(pow(msg->linear.x,2) + pow(msg->linear.y,2))) << endl;
}

void LogManager::velRCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : LOG VEL_R " << std::to_string(sqrt(pow(msg->linear.x,2) + pow(msg->linear.y,2))) << endl;
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

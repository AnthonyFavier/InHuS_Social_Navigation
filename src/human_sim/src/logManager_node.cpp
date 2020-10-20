#include "logManager.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("human_sim");
	log_file_.open(path_ + "/log.txt");

	sub_log_ = nh_.subscribe("log", 100, &LogManager::logCallback, this);
}

LogManager::~LogManager()
{
	log_file_.close();
}

void LogManager::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_ << msg->data;
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

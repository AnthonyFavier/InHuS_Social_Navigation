#include "logManager.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("human_sim");
	log_file_.open(path_ + "/logs/log.txt");
	log_file_ << "LOG STARTS : " << ros::Time::now() << endl;

	sub_log_ = nh_.subscribe("log", 100, &LogManager::logCallback, this);
}

LogManager::~LogManager()
{
	log_file_.close();
}

void LogManager::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : " <<  msg->data << endl;
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

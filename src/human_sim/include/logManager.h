#ifndef DEF_LOGMANAGER
#define DEF_LOGMANAGER

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include <fstream>

using namespace std;

class LogManager
{
public:
	LogManager(ros::NodeHandle nh);
	~LogManager();
	void logCallback(const std_msgs::String::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_log_;
	ofstream log_file_;
	string path_;
};

#endif

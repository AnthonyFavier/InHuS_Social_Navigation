#ifndef DEF_LOGMANAGER
#define DEF_LOGMANAGER

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <math.h>

using namespace std;

class LogManager
{
public:
	LogManager();
	~LogManager();
	void logCallback(const std_msgs::String::ConstPtr& msg);
	void velHCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void velRCallback(const geometry_msgs::Twist::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_log_;
	ros::Subscriber sub_vel_H_;
	ros::Subscriber sub_vel_R_;
	ofstream log_file_;
	string path_;
};

#endif

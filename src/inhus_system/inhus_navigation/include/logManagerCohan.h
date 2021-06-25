#ifndef DEF_LOGMANAGERCOHAN
#define DEF_LOGMANAGERCOHAN

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include <math.h>

using namespace std;

class LogManagerCohan
{
public:
	LogManagerCohan();
	~LogManagerCohan();

	void logCallback(const std_msgs::String::ConstPtr& msg);
	void poseRCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
	ros::NodeHandle nh_;
	string path_;

	ofstream log_file_;
	ros::Subscriber sub_log_;
	ros::Subscriber sub_pose_R_;
};

#endif

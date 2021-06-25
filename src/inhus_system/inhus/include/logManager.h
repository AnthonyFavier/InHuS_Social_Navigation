#ifndef DEF_LOGMANAGER
#define DEF_LOGMANAGER

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
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

	void poseHCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	void poseRCallback(const geometry_msgs::Pose2D::ConstPtr& msg);

private:
	ros::NodeHandle nh_;
	string path_;

	ofstream log_file_inhus_;
	ros::Subscriber sub_log_;
	ros::Subscriber sub_vel_H_;
	ros::Subscriber sub_vel_R_;

	ofstream log_file_inhus_poses_;
	ros::Subscriber sub_pose_H_;
	ros::Subscriber sub_pose_R_;
};

#endif

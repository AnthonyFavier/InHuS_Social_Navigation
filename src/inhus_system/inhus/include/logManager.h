#ifndef DEF_LOGMANAGER
#define DEF_LOGMANAGER

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <fstream>
#include <math.h>
#include "inhus/PoseVel.h"

using namespace std;

class LogManager
{
public:
	LogManager();
	~LogManager();

	void logCallback(const std_msgs::String::ConstPtr& msg);
	void hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);
	void rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);

private:
	ros::NodeHandle nh_;
	string path_;

	ofstream log_file_inhus_;
	ros::Subscriber sub_log_;
	ros::Subscriber sub_h_pose_vel_;
	ros::Subscriber sub_r_pose_vel_;
	ofstream log_file_inhus_poses_;
};

#endif

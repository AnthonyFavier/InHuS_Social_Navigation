#ifndef DEF_POSELOG
#define DEF_POSELOG

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <fstream>

using namespace std;

class PoseLog
{
public:
	PoseLog();
	~PoseLog();
	void poseHCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
	void poseRCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_pose_H_;
	ros::Subscriber sub_pose_R_;
	ofstream log_file_;
	string path_;
};

#endif

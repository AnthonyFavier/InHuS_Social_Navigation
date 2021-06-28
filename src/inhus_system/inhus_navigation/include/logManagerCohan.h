#ifndef DEF_LOGMANAGERCOHAN
#define DEF_LOGMANAGERCOHAN

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include <fstream>
#include <math.h>

using namespace std;

class LogManagerCohan
{
public:
	LogManagerCohan();
	~LogManagerCohan();

	void logCallback(const std_msgs::String::ConstPtr& msg);
	void odomRCB(const nav_msgs::Odometry::ConstPtr& msg);
	void odomHCB(const nav_msgs::Odometry::ConstPtr& msg);
	void robotGoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void robotGoalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

private:
	ros::NodeHandle nh_;
	string path_;

	nav_msgs::Odometry odom_h_;
	nav_msgs::Odometry odom_r_;

	bool goal_received_;
	bool goal_done_;

	int id_;

	ofstream log_file_r_;
	ofstream log_file_h_;
	ros::Subscriber sub_log_;
	ros::Subscriber sub_odom_r_;
	ros::Subscriber sub_odom_h_;

	ros::Subscriber sub_robot_goal_status_;
	ros::Subscriber sub_robot_goal_;
};

#endif

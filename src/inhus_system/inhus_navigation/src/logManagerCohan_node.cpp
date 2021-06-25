#include "logManagerCohan.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManager::LogManager()
{
	path_ = ros::package::getPath("inhus");

	log_file_.open(path_ + "/logs/log_data/log_cohan.txt");
	sub_log_ = nh_.subscribe("/move_base/HATebLocalPlannerROS/hateb_log", 100, &LogManager::logCallback, this);
	sub_pose_R_ = nh_.subscribe("/odom", 100, &LogManager::poseRCallback, this);
}

LogManager::~LogManager()
{
	log_file_.close();
}

void LogManager::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : " <<  msg->data << endl;
}


void LogManager::poseRCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : Pose: x= " << msg->pose.pose.position.x << ", y= " << msg->pose.pose.position.y << ","<< endl;
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

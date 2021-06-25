#include "logManagerCohan.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManagerCohan::LogManagerCohan()
{
	path_ = ros::package::getPath("inhus");

	log_file_.open(path_ + "/logs/log_data/log_cohan.txt");
	sub_log_ = nh_.subscribe("/move_base/HATebLocalPlannerROS/hateb_log", 100, &LogManagerCohan::logCallback, this);
	sub_pose_R_ = nh_.subscribe("/odom", 100, &LogManagerCohan::poseRCallback, this);
}

LogManagerCohan::~LogManagerCohan()
{
	log_file_.close();
}

void LogManagerCohan::logCallback(const std_msgs::String::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : " <<  msg->data << endl;
}


void LogManagerCohan::poseRCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : Pose: x= " << msg->pose.pose.position.x << ", y= " << msg->pose.pose.position.y << ","<< endl;
}

//////////////////////// MAIN ///////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LogManagerCohan");

	LogManagerCohan log_cohan_manager;

	ros::spin();

	return 0;
}

/////////////////////////////////////////////////////////////

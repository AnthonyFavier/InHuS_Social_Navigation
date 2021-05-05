#include "poseLog.h"

//////////////////// POSE LOG ////////////////////////////

// subscribes to Agent positions and sends them to the logManager to be saved

PoseLog::PoseLog()
{
	path_ = ros::package::getPath("human_sim");
	log_file_.open(path_ + "/logs/log_data/poseLog.txt");
	log_file_ << "LOG STARTS : " << ros::Time::now() << endl;

	sub_pose_H_ = nh_.subscribe("interface/human_pose", 100, &PoseLog::poseHCallback, this);
	sub_pose_R_ = nh_.subscribe("interface/robot_pose", 100, &PoseLog::poseRCallback, this);
}

PoseLog::~PoseLog()
{
	log_file_.close();
}

void PoseLog::poseHCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : H " << std::to_string(msg->header.stamp.toSec()) << " " << msg->pose.position.x << " " << msg->pose.position.y << endl;
}

void PoseLog::poseRCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	log_file_ << ros::Time::now() << " : R " << std::to_string(msg->header.stamp.toSec()) << " " << msg->pose.position.x << " " << msg->pose.position.y << endl;
}

//////////////////////// MAIN ///////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "logManager");

	PoseLog log_manager;

	ros::spin();

	return 0;
}

/////////////////////////////////////////////////////////////

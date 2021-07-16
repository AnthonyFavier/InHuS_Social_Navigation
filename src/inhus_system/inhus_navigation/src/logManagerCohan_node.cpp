#include "logManagerCohan.h"

//////////////////// LOG MANAGER ////////////////////////////

LogManagerCohan::LogManagerCohan()
{
	path_ = ros::package::getPath("inhus");

	id_ = 0;

	sub_odom_r_ = nh_.subscribe("/robot_odom", 100, &LogManagerCohan::odomRCB, this);
	sub_odom_h_ = nh_.subscribe("/human_odom", 100, &LogManagerCohan::odomHCB, this);
	sub_robot_goal_ = nh_.subscribe("/robot_goal", 100, &LogManagerCohan::robotGoalCB, this);
	sub_robot_goal_status_ = nh_.subscribe("/robot_goal_status", 100, &LogManagerCohan::robotGoalStatusCB, this);

	goal_received_ = false;
	goal_done_ = true;
}

LogManagerCohan::~LogManagerCohan()
{
	log_file_r_.close();
	log_file_h_.close();
}

void LogManagerCohan::odomRCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_r_ = *msg;

	if(goal_received_ && !goal_done_)
		log_file_r_ << "pose: " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.orientation.z << ", " << msg->pose.pose.orientation.w << ", vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ", " << msg->twist.twist.angular.z << endl;
}

void LogManagerCohan::odomHCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_h_ = *msg;

	if(goal_received_ && !goal_done_)
		log_file_h_ << "pose: " << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.orientation.z << ", " << msg->pose.pose.orientation.w << ", vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ", " << msg->twist.twist.angular.z << endl;
}

void LogManagerCohan::robotGoalCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(!goal_done_)
	{
		log_file_r_.close();
		log_file_h_.close();
		id_++;
	}

	log_file_r_.open(path_ + "/logs/cohan_logs/log_cohan_" + std::to_string(id_) + "_r.txt");
	log_file_r_ << "start: x= " << odom_r_.pose.pose.position.x << ", y= " << odom_r_.pose.pose.position.y << ", z= " << odom_r_.pose.pose.orientation.z << ", w= " << odom_r_.pose.pose.orientation.w << endl;
	log_file_r_ << "goal: x= " << msg->pose.position.x << ", y= " << msg->pose.position.y << ", z= " << msg->pose.position.z << ", w= " << msg->pose.orientation.w << endl;

	log_file_h_.open(path_ + "/logs/cohan_logs/log_cohan_" + std::to_string(id_) + "_h.txt");
	log_file_h_ << "start: x= " << odom_h_.pose.pose.position.x << ", y= " << odom_h_.pose.pose.position.y << ", z= " << odom_h_.pose.pose.orientation.z << ", w= " << odom_h_.pose.pose.orientation.w << endl;

	goal_received_ = true;
	goal_done_ = false;
}

void LogManagerCohan::robotGoalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	if(!msg->status_list.empty())
	{
		if(!goal_done_
		&& ((msg->status_list.back().status == 2) 	// PREEMPTED
		|| (msg->status_list.back().status == 3) 		// SUCCEEDED
		|| (msg->status_list.back().status == 4))) 	// ABORTED
		{
			log_file_r_.close();
			log_file_h_.close();
			goal_done_ = true;
			goal_received_ = false;
			id_++;
		}
	}
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

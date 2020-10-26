#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/CancelGoalAndStop.h"
#include "actionlib_msgs/GoalID.h"

ros::Publisher pub_cmd_vel;
ros::Publisher pub_cancel_goal;

void teleopCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

}

void perturbatedCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub_cmd_vel.publish(*msg);
}

bool cancelGoalAndStop(human_sim::CancelGoalAndStop::Request &req, human_sim::CancelGoalAndStop::Response &res)
{
	actionlib_msgs::GoalID goal_id;
	goal_id.stamp = ros::Time::now();
	pub_cancel_goal.publish(goal_id);

	geometry_msgs::Twist cmd;
	cmd.linear.x = 	0;
	cmd.linear.y = 	0;
	cmd.linear.z = 	0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;
	cmd.angular.z = 0;
	pub_cmd_vel.publish(cmd);

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;

	ros::Subscriber sub_teleop = 		nh.subscribe("controller/teleop_cmd", 100, teleopCmdCallback);
	ros::Subscriber sub_perturbated_cmd = 	nh.subscribe("controller/perturbated_cmd", 100, perturbatedCmdCallback);

	pub_cmd_vel = 		nh.advertise<geometry_msgs::Twist>("/human_cmd_vel", 100);
	pub_cancel_goal =	nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);

	ros::ServiceServer service_cancel_goal_and_stop = nh.advertiseService("cancel_goal_and_stop", cancelGoalAndStop);

	ros::spin();

	return 0;
}

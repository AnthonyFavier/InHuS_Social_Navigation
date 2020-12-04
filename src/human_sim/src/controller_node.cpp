#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "human_sim/CancelGoalAndStop.h"
#include "actionlib_msgs/GoalID.h"
#include "human_sim/Goal.h"

ros::Publisher pub_cmd_vel;
ros::Publisher pub_cancel_goal;
ros::Publisher pub_goal_done;

bool delay_stop_required=false;
ros::Time time_delay_stop_required;

geometry_msgs::Twist cmd_zero;

void teleopCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub_cmd_vel.publish(*msg);
}

void perturbedCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub_cmd_vel.publish(*msg);
}

bool cancelGoalAndStop(human_sim::CancelGoalAndStop::Request &req, human_sim::CancelGoalAndStop::Response &res)
{
	actionlib_msgs::GoalID goal_id;
	goal_id.stamp = ros::Time::now();
	pub_cancel_goal.publish(goal_id);

	pub_goal_done.publish(human_sim::Goal());

	pub_cmd_vel.publish(cmd_zero);

	delay_stop_required = true;
	time_delay_stop_required = ros::Time::now();

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;

	ros::Subscriber sub_teleop = 		nh.subscribe("/boss/human/teleoperation", 100, teleopCmdCallback);
	ros::Subscriber sub_perturbed_cmd = 	nh.subscribe("perturbed_cmd", 100, perturbedCmdCallback);

	pub_cmd_vel = 		nh.advertise<geometry_msgs::Twist>("interface/out/final_cmd", 100);
	pub_cancel_goal =	nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
	pub_goal_done = 	nh.advertise<human_sim::Goal>("goal_done", 100);

	ros::ServiceServer service_cancel_goal_and_stop = nh.advertiseService("cancel_goal_and_stop", cancelGoalAndStop);

	ros::Duration delay(0.3);
	ros::Rate rate(15);

	cmd_zero.linear.x = 0;
	cmd_zero.linear.y = 0;
	cmd_zero.linear.z = 0;
	cmd_zero.angular.x = 0;
	cmd_zero.angular.y = 0;
	cmd_zero.angular.z = 0;

	while(ros::ok())
	{
		if(delay_stop_required && ros::Time::now() - time_delay_stop_required > delay)
		{
			pub_cmd_vel.publish(cmd_zero);
			delay_stop_required = false;
		}

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

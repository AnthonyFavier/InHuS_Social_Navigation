#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub_cmd_vel;

void teleopCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

}

void perturbatedCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	pub_cmd_vel.publish(*msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;

	ros::Subscriber sub_teleop = 		nh.subscribe("controller/teleop_cmd", 100, teleopCmdCallback);
	ros::Subscriber sub_perturbated_cmd = 	nh.subscribe("controller/perturbated_cmd", 100, perturbatedCmdCallback);

	pub_cmd_vel = 	nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

	ros::spin();

	return 0;
}

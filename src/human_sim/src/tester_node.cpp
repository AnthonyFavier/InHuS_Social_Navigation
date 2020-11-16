#include "ros/ros.h"
#include "human_sim/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>

#include <iostream>

#define PI 3.14159265358

using namespace std;

struct GoalArea
{
	human_sim::Goal goal;
	float radius;
};

geometry_msgs::PoseStamped getPose(human_sim::Goal goal)
{
	geometry_msgs::PoseStamped pose;

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";

	pose.pose.position.x = goal.x;
	pose.pose.position.y = goal.y;

	tf2::Quaternion q;
	q.setRPY(0,0,goal.theta);
	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();

	return pose;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "testeur");

	ros::NodeHandle nh;

	int choice = 0;
	GoalArea area;
	geometry_msgs::PoseStamped pose;
	vector<GoalArea> goals;

	int choice_init = 0;

	area.goal.type="Position";

	string topic_robot = "/move_base_simple/goal";

	cout << "1- Simple move_base" << endl << "2- HATEB" << "which ? ";
	cin >> choice;

	if(choice == 1)
		topic_robot = "/robot" + topic_robot;

	ros::Publisher pub_goal_human = nh.advertise<human_sim::Goal>("/boss/human/new_goal", 1);
	ros::Publisher pub_goal_robot = nh.advertise<geometry_msgs::PoseStamped>(topic_robot, 1);

	//0// to have easier index
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	//1//
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);
	//2//
	area.goal.x=3.15; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	//3//
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	goals.push_back(area);
	//4//
	area.goal.x=7.90; 	area.goal.y=5.1; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);
	//5//
	area.goal.x=7.8; 	area.goal.y=9.98; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);
	//6//
	area.goal.x=3.42; 	area.goal.y=9.48; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	//7//
	area.goal.x=4.72; 	area.goal.y=17.68; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	//8//
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	goals.push_back(area);
	//9//
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);
	//10//
	area.goal.x=1.15; 	area.goal.y=6.52; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);

	while(ros::ok())
	{
		cout << endl << endl << endl << endl << endl << endl << endl << endl << endl << endl;
		cout << "1- Human" << endl << "2- Robot" << endl << "3- Scenario" << endl << "Choice ? ";
		cin >> choice;

		if(choice == 1)
		{
			cout <<  endl << "Which goal [1-10] ? ";
			cin >> choice;

			if(goals[choice].radius == 0)
				pub_goal_human.publish(goals[choice].goal);
		}
		else if(choice == 2)
		{
			cout <<  endl << "Which goal [1-10] ? ";
			cin >> choice;

			if(goals[choice].radius == 0)
			{
				pose = getPose(goals[choice].goal);
				pub_goal_robot.publish(pose);
			}
		}
		else if(choice == 3)
		{
			cout << endl << "1- cross middle R4 H1" << endl << "2- corridor R8 H3" << endl << "3- cross R8 H9" << endl << "4- narrow corridor R9 H1" << endl << "Which scenario [1-4] ? ";
			cin >> choice;

			cout << endl << "1- init" << endl << "2- start" << endl << "Choice ? ";
			cin >> choice_init;

			switch(choice)
			{
				case 1:
					// R4 H1 => R1 H4 (R delayed)
					if(choice_init == 1)
					{
						pose = getPose(goals[4].goal);
						pub_goal_robot.publish(pose);
						pub_goal_human.publish(goals[1].goal);
					}
					else
					{
						cout << "Delay ? ";
						float delay;
						cin >> delay;

						pub_goal_human.publish(goals[4].goal);

						ros::Duration(delay).sleep();

						pose = getPose(goals[1].goal);
						pub_goal_robot.publish(pose);
					}
					break;
				case 2:
					// R8 H3 => R3 H8 (R delayed)
					if(choice_init == 1)
					{
						pose = getPose(goals[8].goal);
						pub_goal_robot.publish(pose);
						pub_goal_human.publish(goals[3].goal);
					}
					else
					{
						cout << "Delay ? (2)";
						float delay;
						cin >> delay;

						pub_goal_human.publish(goals[8].goal);

						ros::Duration(delay).sleep();

						pose = getPose(goals[3].goal);
						pub_goal_robot.publish(pose);
					}
					break;
				case 3:
					// R8 H9 => R9 H8 (R delayed)
					if(choice_init == 1)
					{
						pose = getPose(goals[8].goal);
						pub_goal_robot.publish(pose);
						pub_goal_human.publish(goals[9].goal);
					}
					else
					{
						cout << "Delay ? (1)";
						float delay;
						cin >> delay;

						pub_goal_human.publish(goals[8].goal);

						ros::Duration(delay).sleep();

						pose = getPose(goals[9].goal);
						pub_goal_robot.publish(pose);
					}
					break;

				case 4:
					// R9 H1 => R1 H9 (R delayed)
					if(choice_init == 1)
					{
						pose = getPose(goals[9].goal);
						pub_goal_robot.publish(pose);
						pub_goal_human.publish(goals[1].goal);
					}
					else
					{
						cout << "Delay ? (1)";
						float delay;
						cin >> delay;

						pub_goal_human.publish(goals[9].goal);

						ros::Duration(delay).sleep();

						pose = getPose(goals[1].goal);
						pub_goal_robot.publish(pose);
					}
					break;

				default:
					break;
			}
		}
	}
}

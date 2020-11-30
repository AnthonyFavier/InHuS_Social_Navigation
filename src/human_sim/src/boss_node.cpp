#include "ros/ros.h"
#include "human_sim/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int32.h>
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

void cleanInput()
{
	cout << "Wrong input..." << endl << endl;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

void wait(float delay)
{
	if(delay > 0)
	{
		int length = 10;
		cout << "[";
		for(int i=0; i<length; i++)
			cout << "#";
		cout << "]";

		ros::Time start = ros::Time::now();
		ros::Duration delay_d(delay);

		ros::Rate rate(5);
		bool continuer = true;
		float speed = 10/delay;
		int nb;
		string str;

		while(ros::ok() && continuer)
		{
			ros::Duration passed = ros::Time::now() - start;

			nb = length - speed*passed.toSec() +1 ;
			str = "[";
			for(int i=0; i<nb; i++)
				str = str + "#";
			for(int i=nb; i<length; i++)
				str = str + ".";
			str = str + "]";
			cout << "\r" << str << flush;

			if(passed > delay_d)
				continuer = false;
			else
				rate.sleep();
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boss");

	ros::NodeHandle nh;

	int choice = 0;
	int choice_init = 0;
	float delay;
	GoalArea area;
	geometry_msgs::PoseStamped pose;
	vector<GoalArea> goals;

	area.goal.type="Position";

	string topic_robot = "/move_base_simple/goal";
	while(ros::ok() && (cout 	<< "1- Simple move_base" << endl 
					<< "2- HATEB" << endl 
					<< "Choice ? ")
	&& (!(cin >> choice) || !(choice>=1 && choice<=2)))
		cleanInput();
	if(choice == 1)
		topic_robot = "/robot" + topic_robot;
	ros::Publisher pub_goal_robot = nh.advertise<geometry_msgs::PoseStamped>(topic_robot, 1);
	ros::Publisher pub_goal_human = 	nh.advertise<human_sim::Goal>("/boss/human/new_goal", 1);
	ros::Publisher pub_operating_mode = 	nh.advertise<std_msgs::Int32>("/boss/human/operating_mode", 1);
	ros::Publisher pub_set_behavior = 	nh.advertise<std_msgs::Int32>("/boss/human/set_behavior", 1);

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

	//9 bis // 11
	area.goal.x=2.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);	
	//4 bis // 12
	area.goal.x=7.50; 	area.goal.y=7.32; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	// 13
	area.goal.x=10.4; 	area.goal.y=2.60; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	// 14
	area.goal.x=10.4; 	area.goal.y=3.60; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	// 15
	area.goal.x=10.4; 	area.goal.y=13.5; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	// 16
	area.goal.x=0.8; 	area.goal.y=3.9; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);	
	// 17
	area.goal.x=0.8; 	area.goal.y=4.9; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	// 18
	area.goal.x=0.8; 	area.goal.y=13.5; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	while(ros::ok())
	{
		for(int i=0; i<10; i++){cout << endl;}
		while(ros::ok() && (cout	<< "1- Human goal" << endl 
						<< "2- Robot goal" << endl 
						<< "3- Scenario" << endl 
						<< "4- Operating mode" << endl
						<< "5- Set Behavior" << endl
						<< "Choice ? ")
		&& (!(cin >> choice) || !(choice>=1 && choice<=5)))
			cleanInput();

		switch(choice)
		{
			/* HUMAN GOAL */
			case 1:
				while(ros::ok() && (cout <<  endl << "Which goal [1-10] ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=10)))
					cleanInput();

				if(goals[choice].radius == 0)
					pub_goal_human.publish(goals[choice].goal);
				break;

			/* ROBOT GOAL */
			case 2:
				while(ros::ok() && (cout <<  endl << "Which goal [1-10] ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=10)))
					cleanInput();

				if(goals[choice].radius == 0)
					pub_goal_robot.publish(getPose(goals[choice].goal));
				break;

			/* SCENARIO */
			case 3:
			{
				int h_goal, r_goal;

				// Select scenario
				while(ros::ok() && (cout << endl	<< "1- cross middle R4 H1" << endl 
									<< "2- corridor R8 H3" << endl 
									<< "3- cross R8 H9" << endl 
									<< "4- narrow corridor R9 H1" << endl 
									<< "Which scenario [1-4] ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=4)))
					cleanInput();

				// Init or Start
				while((cout << endl 	<< "1- init" << endl 
							<< "2- start" << endl 
							<< "Choice ? ")
				&& (!(cin >> choice_init) || !(choice_init>=1 && choice_init<=2)))
					cleanInput();

				// Get delay if start
				if(choice_init == 2)
				{
					while(ros::ok() && (cout << "Delay ? ")
					&& (!(cin >> delay)))
						cleanInput();
				}

				switch(choice)
				{
					case 1:
						// cross middle
						if(choice_init == 1)
							{r_goal = 12; h_goal = 1;}
						else
							{r_goal = 1; h_goal = 4;}
						break;

					case 2:
						// corridor
						if(choice_init == 1)
							{r_goal = 15; h_goal = 13;}
						else
							{r_goal = 14; h_goal = 15;}
						break;

					case 3:
						// cross
						if(choice_init == 1)
							{r_goal = 8; h_goal = 9;}
						else
							{r_goal = 11; h_goal = 8;}
						break;

					case 4:
						// narrow corridor
						if(choice_init == 1)
							{r_goal = 18; h_goal = 16;}
						else
							{r_goal = 17; h_goal = 18;}
						break;

					default:
						break;
				}

				// Publish goals
				if(delay>=0)
				{
					cout << "Publish goal : human" << endl;
					pub_goal_human.publish(goals[h_goal].goal);
					wait(delay);
					cout << "Publish goal : robot" << endl;
					pub_goal_robot.publish(getPose(goals[r_goal].goal));
				}
				else
				{
					cout << "Publish goal : robot" << endl;
					pub_goal_robot.publish(getPose(goals[r_goal].goal));
					wait(-delay);
					cout << "Publish goal : human" << endl;
					pub_goal_human.publish(goals[h_goal].goal);
				}
				break;
			}

			/* OPERATING MODE */
			case 4:
			{
				while(ros::ok() && (cout <<  endl 	<< "1- AUTONOMOUS" << endl
									<< "2- SPECIFIED" << endl
									<< "Choice ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=2)))
					cleanInput();
	
				std_msgs::Int32 operating_mode;

				switch(choice)
				{
					case 1:
						operating_mode.data = 0;
						break;
					case 2:
						operating_mode.data = 1;
						break;
				}
				pub_operating_mode.publish(operating_mode);
				break;
			}

			/* SET BEHAVIOR */
			case 5:
			{
				while(ros::ok() && (cout <<  endl 	<< "1- NONE" << endl
									<< "2- RANDOM" << endl
									<< "3- STOP_LOOK" << endl
									<< "4- HARASS" << endl
									<< "Choice ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=4)))
					cleanInput();

				std_msgs::Int32 set_behavior;

				switch(choice)
				{
					case 1:
						set_behavior.data = 0;
						break;
					case 2:
						set_behavior.data = 1;
						break;
					case 3:
						set_behavior.data = 2;
						break;
					case 4:
						set_behavior.data = 3;
						break;
				}
				pub_set_behavior.publish(set_behavior);
				break;
			}

			default:
				break;
		}
	}
}


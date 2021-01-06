#include "ros/ros.h"
#include "human_sim/Goal.h"
#include <tf2/LinearMath/Quaternion.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int32.h>
#include <vector>

#include <boost/thread/thread.hpp>

#include <iostream>

#define PI 3.14159265358

using namespace std;

struct GoalArea
{
	human_sim::Goal goal;
	float radius;
};

bool autoRobotGoal;
GoalArea area;
vector<GoalArea> goals;
int current_behavior;

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
		cout << endl;
	}
}

void pubGoalsRobot(ros::Publisher& pub_goal_robot)
{
	ros::Duration loop(10);
	while(ros::ok())
	{
		if(autoRobotGoal)
		{
			int i = rand()%10 + 1;
			cout << "plop " << i << endl;
			pub_goal_robot.publish(getPose(goals[i].goal));
		}
		loop.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boss");

	ros::NodeHandle nh;

	int choice = 0;
	int choice_init = 0;
	float delay;
	geometry_msgs::PoseStamped pose;
	current_behavior = 0;

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
	ros::Publisher pub_set_behavior = 	nh.advertise<std_msgs::Int32>("/boss/human/set_behavior", 1);

	area.goal.type="Position";

	//0// to have easier index
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
	area.goal.x=0.65; 	area.goal.y=8.50; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);

	// Scenarios //
	// Cross
	//11 // init cross H
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	goals.push_back(area);	
	//12 // goal cross H
	area.goal.x=9.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	goals.push_back(area);	
	//13 // init cross R
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);	
	//14 // goal cross R
	area.goal.x=2.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	goals.push_back(area);	
	
	// Narrow Passage //
	//15 // init narrow passage H
	area.goal.x=1.0; 	area.goal.y=1.9; 	area.goal.theta=0;	area.radius=0;
	goals.push_back(area);	
	/////// goal narrow passage H => 4
	//16 // init narrow passage R
	area.goal.x=7.50; 	area.goal.y=7.32; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);
	/////// goal narrow passage R => 1

	// Corridor //
	//17 // init corridor H
	area.goal.x=10.4; 	area.goal.y=2.60; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);	
	//18 // goal corridor H
	area.goal.x=10.4; 	area.goal.y=12.5; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	//19 // init corridor R
	area.goal.x=10.4; 	area.goal.y=13.5; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);
	//20 // goal corridor R
	area.goal.x=10.4; 	area.goal.y=3.60; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	// Narrow corridor //
	//21 // init narrow corridor H
	area.goal.x=0.8; 	area.goal.y=3.9; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);	
	//22 // goal narrow corridor H
	area.goal.x=0.8; 	area.goal.y=12.5; 	area.goal.theta=PI/2;	area.radius=0;
	goals.push_back(area);
	//23 // init narrow corridor R
	area.goal.x=0.8; 	area.goal.y=13.5; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);	
	//24 // goal narrow corridor R
	area.goal.x=0.8; 	area.goal.y=4.9; 	area.goal.theta=-PI/2;	area.radius=0;
	goals.push_back(area);

	// spawn thread to publish auto goals to robot
	autoRobotGoal = false;
	boost::thread thread_b(pubGoalsRobot, pub_goal_robot);

	while(ros::ok())
	{
		for(int i=0; i<10; i++){cout << endl;}
		cout << "current auto send robot goal : ";
		if(autoRobotGoal)
			cout << "True";
		else
			cout << "False";
		cout << endl << "current behavior : ";
		switch(current_behavior)
		{
			case 0:
				cout << "NONE" << endl;
				break;
			case 1:
				cout << "NON_STOP" << endl;
				break;
			case 2:
				cout << "RANDOM" << endl;
				break;
			case 3:
				cout << "STOP_LOOK" << endl;
				break;
			case 4:
				cout << "HARASS" << endl;
				break;

		}
		while(ros::ok() && (cout	<< "1- Human goal" << endl 
						<< "2- Robot goal" << endl 
						<< "3- Scenario" << endl 
						<< "4- Set Behavior" << endl
						<< "5- Auto goals robot" << endl
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
				while(ros::ok() && (cout << endl	<< "1- cross" << endl 
									<< "2- narrow passage" << endl 
									<< "3- corridor" << endl 
									<< "4- narrow corridor" << endl 
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
				else
					delay=0;

				switch(choice)
				{
					case 1:
						// cross
						if(choice_init == 1)
							{h_goal = 11; r_goal = 13;}
						else
							{h_goal = 12; r_goal = 14;}
						break;

					case 2:
						// narrow passage
						if(choice_init == 1)
							{h_goal = 15; r_goal = 16;}
						else
							{h_goal = 4; r_goal = 1;}
						break;

					case 3:
						// corridor
						if(choice_init == 1)
							{h_goal = 17; r_goal = 19;}
						else
							{h_goal = 18; r_goal = 20;}
						break;

					case 4:
						// narrow corridor
						if(choice_init == 1)
							{h_goal = 21; r_goal = 23;}
						else
							{h_goal = 22; r_goal = 24;}
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

			/* SET BEHAVIOR */
			case 4:
			{
				while(ros::ok() && (cout <<  endl 	<< "1- NONE" << endl
									<< "2- NON_STOP" << endl
									<< "3- RANDOM" << endl
									<< "4- STOP_LOOK" << endl
									<< "5- HARASS" << endl
									<< "Choice ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=5)))
					cleanInput();

				std_msgs::Int32 set_behavior;

				switch(choice)
				{
					case 1:
						current_behavior = 0;
						break;
					case 2:
						current_behavior = 1;
						break;
					case 3:
						current_behavior = 2;
						break;
					case 4:
						current_behavior = 3;
						break;
					case 5:
						current_behavior = 4;
						break;
				}
				set_behavior.data = current_behavior;
				pub_set_behavior.publish(set_behavior);
				break;
			}

			/* AUTOMATICALLY SEND GOALS TO ROBOT */
			case 5:
				while(ros::ok() && (cout << endl	<< "1- True" << endl
									<< "2- False" << endl
									<< "Choice ? ? ")
				&& (!(cin >> choice) || !(choice>=1 && choice<=2)))
					cleanInput();

				switch(choice)
				{
					case 1:
						autoRobotGoal = true;
						break;
					case 2:
						autoRobotGoal = false;
						break;
				}

				break;

			default:
				break;
		}
	}
}


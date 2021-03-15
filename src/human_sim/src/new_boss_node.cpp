#include "boss.h"

//////////////////// AGENT MANAGER //////////////////////

AgentManager::AgentManager(string name, string topic_goal)
{
	name_ = name;
	topic_goal_ = topic_goal;
}

string AgentManager::getName()
{
	 return name_;
}

//////////////////// HUMAN MANAGER //////////////////////

HumanManager::HumanManager(string name, string topic_goal) : AgentManager(name, topic_goal)
{
	pub_goal_ =	nh_.advertise<human_sim::Goal>(topic_goal_, 1);
}

void HumanManager::publishGoal(GoalArea goal)
{
	pub_goal_.publish(goal.goal);
}

//////////////////// ROBOT MANAGER //////////////////////

RobotManager::RobotManager(string name, string topic_goal) : AgentManager(name, topic_goal)
{
	pub_goal_ =	nh_.advertise<geometry_msgs::PoseStamped>(topic_goal_, 1);
}

void RobotManager::publishGoal(GoalArea goal)
{
	pub_goal_.publish(this->getPose(goal.goal));
}

geometry_msgs::PoseStamped RobotManager::getPose(human_sim::Goal goal)
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

/////////////////////////////////////////////////////////
///////////////////////// BOSS //////////////////////////

Boss::Boss()
{
	// init goals
	GoalArea area;
	area.goal.type="Position";

	//0// to have easier index
	list_goals_.push_back(area);

	//1//
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//2//
	area.goal.x=3.15; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//3//
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//4//
	area.goal.x=7.90; 	area.goal.y=5.1; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//5//
	area.goal.x=7.8; 	area.goal.y=9.98; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//6//
	area.goal.x=3.42; 	area.goal.y=9.48; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//7//
	area.goal.x=4.72; 	area.goal.y=17.68; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//8//
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//9//
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//10//
	area.goal.x=0.65; 	area.goal.y=8.50; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);

	// Scenarios //
	// Wide Area
	//11 // init wide area H
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//12 // goal wide are H
	area.goal.x=9.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//13 // init wide area R
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//14 // goal wide area R
	area.goal.x=2.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);

	// Narrow Passage //
	//15 // init narrow passage H
	area.goal.x=1.0; 	area.goal.y=1.9; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	/////// goal narrow passage H => 4
	//16 // init narrow passage R
	area.goal.x=7.50; 	area.goal.y=7.32; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	/////// goal narrow passage R => 1

	// Corridor //
	//17 // init corridor H
	area.goal.x=10.35; 	area.goal.y=2.60; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//18 // goal corridor H
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);
	//19 // init corridor R
	area.goal.x=10.35; 	area.goal.y=14.3; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//20 // goal corridor R
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	list_goals_.push_back(area);

	// Narrow corridor //
	//21 // init narrow corridor H
	area.goal.x=0.65; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//22 // goal narrow corridor H
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	list_goals_.push_back(area);
	//23 // init narrow corridor R
	area.goal.x=0.65; 	area.goal.y=14.1; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);
	//24 // goal narrow corridor R
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	list_goals_.push_back(area);

}

void Boss::appendAgent(AgentManager* agent)
{
	agent_managers_.push_back(agent);
}

void Boss::showState()
{
	cout << "==========================" << endl;
	cout << "=========> BOSS <=========" << endl;
	cout << "==========================" << endl;
	cout << endl;
}

void Boss::askChoice()
{
	// Ask Main choice_
	while(ros::ok() && (cout	<< "1- Send goal" << endl
														<< "2- Scenario" << endl
														<< "3- Set Attitude" << endl
														<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=1 && choice_<=3)))
		cleanInput();
	cout << endl;

	switch(choice_)
	{
		// Send goal
		case 1:
			this->askSendGoal();
			break;

		// Scenario
		case 2:
			this->askScenario();
			break;

		// Set attitude
		case 3:
			this->askSetAttitude();
			break;

		default:
			break;
	}
}

void Boss::askSendGoal()
{
	// Ask which agent to send the goal
	while(ros::ok() && this->showAgents()
									&& (cout 	<< "0- Back" << endl
														<< "Which agent ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=10)))
		cleanInput();
	cout << endl;
	if(choice_==0){return;} // Back

	int choice_agent = choice_-1;

	// Ask goal from list or manual input
	while(ros::ok() && (cout	<< "1- From List" << endl
														<< "2- Enter coordinates" << endl
														<< "0- Back" << endl
														<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=2)))
		cleanInput();
	cout << endl;
	if(choice_==0){return;} // Back

	switch(choice_)
	{
		// From list
		case 1:{
			// Ask which goal from list to send
			while(ros::ok() && (cout 	<< "Select a goal [1-10] " << endl
																<< "0- Back" << endl
																<< "Choice ? ")
			&& (!(cin >> choice_) || !(choice_>=0 && choice_<=10)))
				cleanInput();
			cout << endl;
			if(choice_==0){return;} // Back

			agent_managers_[choice_agent]->publishGoal(list_goals_[choice_]);

			break;}

		// Enter coordinates
		case 2:{
			bool ok = false;
			while(!ok)
			{
				// Ask goal coordinates
				GoalArea goal;
				goal.radius = 0;
				goal.goal.type = "position";
				while(ros::ok() && (cout 	<< "Goal x : ")
				&& (!(cin >> goal.goal.x)))
					cleanInput();
				while(ros::ok() && (cout 	<< "Goal y : ")
				&& (!(cin >> goal.goal.y)))
					cleanInput();
				while(ros::ok() && (cout 	<< "Goal theta : ")
				&& (!(cin >> goal.goal.theta)))
					cleanInput();

				while(ros::ok() && (cout 	<< "Send this goal : (x=" << goal.goal.x << ", y=" << goal.goal.y << ", theta=" << goal.goal.theta << ")" << endl
																	<< "1- Yes" << endl
																	<< "2- No, enter coordinates again" << endl
																	<< "0- Back" << endl
																	<< "Choice ? ")
				&& (!(cin >> choice_) || !(choice_>=0 && choice_<=2)))
					cleanInput();
				cout << endl;
				if(choice_==0){return;} // Back

				if(choice_ == 1)
				{
						agent_managers_[choice_agent]->publishGoal(goal);
						ok = true;
				}
			}

			break;}

		default:
			break;
	}
}

void Boss::askScenario()
{

}

void Boss::askSetAttitude()
{

}

void Boss::cleanInput()
{
	cout << "Wrong input..." << endl << endl;
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	choice_ = 0;
}

bool Boss::showAgents()
{
	for(int i=0	; i<int(agent_managers_.size()); i++)
	{
		cout << i+1 << "- " << agent_managers_[i]->getName() << endl;
	}

	return true;
}

/////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boss");

	Boss boss;
	HumanManager human_manager1("human1", "/boss/human/new_goal");
	boss.appendAgent(&human_manager1);
	RobotManager robot_manager1("robot1", "/move_base_simple/goal");
	boss.appendAgent(&robot_manager1);

	while(ros::ok())
	{
		boss.showState();
		boss.askChoice();
	}
}

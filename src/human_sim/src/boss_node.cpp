#include "boss.h"

//////////////////// AGENT MANAGER //////////////////////

AgentManager::AgentManager(string name)
{
	name_ = name;
}

string AgentManager::getName()
{
	 return name_;
}

Type AgentManager::getType()
{
	return type_;
}

bool AgentManager::isGoalDone()
{
	return goal_done_;
}

//////////////////// HUMAN MANAGER //////////////////////

HumanManager::HumanManager(string name) : AgentManager(name)
{
	type_ = HUMAN;

	string topic_goal = "/boss/" + name_ + "/new_goal";
	pub_goal_ =	nh_.advertise<human_sim::Goal>(topic_goal, 1);

	string topic_attitude = "/boss/" + name_ + "/set_attitude";
	pub_attitude_ = nh_.advertise<std_msgs::Int32>(topic_attitude, 1);

	string topic_manual = "/boss/" + name_ + "/teleoperation";
	pub_manual_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_manual, 1);

	string topic_goal_done = "/" + name_ + "/goal_done";
	sub_goal_done_ = 	nh_.subscribe("/human/goal_done", 1, &HumanManager::goalDoneCB, this);

	string topic_goal_start = "/" + name_ + "/new_goal";
	sub_goal_start_ =	nh_.subscribe("/human/new_goal", 1, &HumanManager::goalStartCB, this);

	goal_done_ = true;
	current_attitude_ = 0;
}

void HumanManager::publishGoal(GoalArea goal)
{
	pub_goal_.publish(goal.goal);
}

void HumanManager::publishManualCmd(geometry_msgs::Twist cmd)
{
	pub_manual_cmd_.publish(cmd);
}

void HumanManager::showState()
{
	cout << "\t\tCurrent attitude : ";
	switch(current_attitude_)
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
}

void HumanManager::setAttitude(int attitude)
{
	std_msgs::Int32 set_attitude;
	current_attitude_ = attitude;
	set_attitude.data = current_attitude_;
	pub_attitude_.publish(set_attitude);
}

void HumanManager::goalDoneCB(const human_sim::Goal::ConstPtr& msg)
{
	goal_done_ = true;
}

void HumanManager::goalStartCB(const human_sim::Goal::ConstPtr& msg)
{
	goal_done_ = false;
}

//////////////////// ROBOT MANAGER //////////////////////

RobotManager::RobotManager(string name, string topic_goal, string topic_goal_status) : AgentManager(name)
{
	type_ = ROBOT;

	pub_goal_ =	nh_.advertise<geometry_msgs::PoseStamped>(topic_goal, 1);

	sub_goal_status_ = nh_.subscribe(topic_goal_status, 1, &RobotManager::goalStatusCB, this);

	goal_done_ = true;
}

void RobotManager::publishGoal(GoalArea goal)
{
	pub_goal_.publish(this->getPose(goal.goal));
}

void RobotManager::showState()
{
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

void RobotManager::goalStatusCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	if(!msg->status_list.empty())
	{
		if(msg->status_list.back().status == 1) // ACTIVE
			goal_done_ = false;
		else if(msg->status_list.back().status == 0) // PENDING
			goal_done_ = false;
		else if(msg->status_list.back().status == 3) // SUCCEEDED
			goal_done_ = true;
		else
			goal_done_ = false;
	}
}

/////////////////////////////////////////////////////////
///////////////////////// BOSS //////////////////////////

Boss::Boss()
: endless_delay_(2)
{
	this->initGoals();

	endless_agent1_ = 0;
	endless_agent2_ = 0;
	endless_agent1_on_ = false;
	endless_agent2_on_ = false;
	endless_agent1_i_ = endless_goals_agent1_.size()-1;
	endless_agent2_i_ = endless_goals_agent2_.size()-1;
}

void Boss::initGoals()
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


	// Endless Agent1 //
	endless_goals_agent1_.push_back(list_goals_[4]);
	area.goal.x=5.4; 	area.goal.y=7.3; 	area.goal.theta=-PI;	area.radius=0;
	endless_goals_agent1_.push_back(area);
	area.goal.x=4.0; 	area.goal.y=9.5; 	area.goal.theta=PI/2;	area.radius=0;
	endless_goals_agent1_.push_back(area);
	area.goal.x=9.0; 	area.goal.y=10.2; 	area.goal.theta=0;	area.radius=0;
	endless_goals_agent1_.push_back(area);

	// Endless Agent2 //
	endless_goals_agent2_.push_back(list_goals_[2]);
	area.goal.x=8.00; 	area.goal.y=7.60; 	area.goal.theta=0;	area.radius=0;
	endless_goals_agent2_.push_back(area);
	area.goal.x=7.5; 	area.goal.y=11.0; 	area.goal.theta=-PI;	area.radius=0;
	endless_goals_agent2_.push_back(area);
	area.goal.x=5.4; 	area.goal.y=11.3; 	area.goal.theta=PI/2;	area.radius=0;
	endless_goals_agent2_.push_back(area);
	area.goal.x=4.3; 	area.goal.y=6.3; 	area.goal.theta=-PI/2;	area.radius=0;
	endless_goals_agent2_.push_back(area);
	area.goal.x=8.00; 	area.goal.y=7.60; 	area.goal.theta=0;	area.radius=0;
	endless_goals_agent2_.push_back(area);
}

void Boss::spawnThreadEndless()
{
	boost::thread thread_endless1(&Boss::threadPubEndlessAgent1, this);
	boost::thread thread_endless2(&Boss::threadPubEndlessAgent2, this);
}

void Boss::appendAgent(AgentManager* agent)
{
	agent_managers_.push_back(agent);
}

void Boss::showState()
{
	cout << endl;
	cout << "==========================" << endl;
	cout << "=========> BOSS <=========" << endl;
	cout << "==========================" << endl;
	cout << endl;

	cout << "State :";
	for(int i=0; i<int(agent_managers_.size()); i++)
	{
		cout << "\tAgent" << i+1 << " (" << agent_managers_[i]->getName() << "):" << endl;
		agent_managers_[i]->showState();
	}

	cout << endl << "\tEndless agent1 : ";
	if(endless_agent1_on_)
		cout << "on" << endl;
	else
		cout << "off" << endl;

	cout << "\tEndless agent2 : ";
	if(endless_agent2_on_)
		cout << "on" << endl;
	else
		cout << "off" << endl;

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
			while(ros::ok() && (cout 	<< "[1-10] Select a goal" << endl
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
	// Default agents (1 and 2) selected if only 2 agents in the scene
	int agent1 = 0;
	int agent2 = 1;

	// Check if there are enough agents in the scene (at least 2)
	if(int(agent_managers_.size())<2)
	{
		cout << "Not enough agents in the scene !" << endl;
		return;
	}
	else if(int(agent_managers_.size())>2)
	{
		// Select agent1
		this->showAgents();
		cout 	<< "0- Back" << endl;
		cout 	<< "Which agent ? " << endl;
		while(ros::ok()	&& (cout << "agent1 : ")
		&& (!(cin >> choice_) || !(choice_>=0 && choice_<=int(agent_managers_.size()))))
			cleanInput();
		if(choice_==0){return;} // Back
		agent1 = choice_-1;

		// Select agent2
		while(ros::ok()	&& (cout << "agent2 : ")
		&& (!(cin >> choice_) || !(choice_>=0 && choice_<=int(agent_managers_.size()))))
			cleanInput();
		cout << endl;
		if(choice_==0){return;} // Back
		agent2 = choice_-1;
	}

	cout << "Selected agents : " << agent_managers_[agent1]->getName() << " & " << agent_managers_[agent2]->getName() << endl;

	// Ask which scenario
	while(ros::ok() && (cout << endl	<< "1- Wide Area" << endl
																		<< "2- Narrow Passage" << endl
																		<< "3- Corridor" << endl
																		<< "4- Narrow Corridor" << endl
																		<< "5- Endless" << endl
																		<< "0- Back" << endl
																		<< "Which scenario ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=5)))
		cleanInput();
	cout << endl;
	if(choice_==0){return;} // Back
	int choice_scenario = choice_;

	// If not endless
	if(choice_scenario!=5)
	{
		// Ask init or start the scenario
		while((cout << endl 	<< "1- Init" << endl
													<< "2- Start" << endl
													<< "0- Back" << endl
													<< "Choice ? ")
		&& (!(cin >> choice_) || !(choice_>=0 && choice_<=2)))
			cleanInput();
		if(choice_==0){return;} // Back
		int choice_init = choice_;

		// Get delay if start
		float delay;
		if(choice_init == 2)
		{
			while(ros::ok() && (cout << "Delay " << agent_managers_[agent2]->getName() << " (s) ? ")
			&& (!(cin >> delay)))
				cleanInput();
		}
		else
			delay=0;

		// Get corresponding goals
		int agent1_goal; int agent2_goal;
		switch(choice_scenario)
		{
			case 1: // Wide Area
				if(choice_init == 1)
					{agent1_goal = 11; agent2_goal = 13;}
				else
					{agent1_goal = 12; agent2_goal = 14;}
				break;

			case 2: // Narrow Passage
				if(choice_init == 1)
					{agent1_goal = 15; agent2_goal = 16;}
				else
					{agent1_goal = 4; agent2_goal = 1;}
				break;

			case 3: // Corridor
				if(choice_init == 1)
					{agent1_goal = 17; agent2_goal = 19;}
				else
					{agent1_goal = 18; agent2_goal = 20;}
				break;

			case 4: // Narrow Corridor
				if(choice_init == 1)
					{agent1_goal = 21; agent2_goal = 23;}
				else
					{agent1_goal = 22; agent2_goal = 24;}
				break;
			}

		// Publish goals
		if(delay>=0)
		{
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(list_goals_[agent1_goal]);
			wait(delay);
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(list_goals_[agent2_goal]);
		}
		else
		{
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(list_goals_[agent2_goal]);
			wait(-delay);
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(list_goals_[agent1_goal]);
		}
	}
	else // endless
	{
		// Set endless agents
		endless_agent1_ = agent1;
		endless_agent2_ = agent2;

		// Ask start or stap endless scenario
		while((cout << endl 	<< "1- Start" << endl
													<< "2- Stop" << endl
													<< "0- Back" << endl
													<< "Choice ? ")
		&& (!(cin >> choice_) || !(choice_>=0 && choice_<=2)))
			cleanInput();
		if(choice_==0){return;} // Back
		int choice_endless = choice_;

		// Ask which agent (1, 2, both)
		while((cout << endl 	<< "1- Both" << endl
													<< "2- Only " << agent_managers_[endless_agent1_]->getName() << endl
													<< "3- Only " << agent_managers_[endless_agent2_]->getName() << endl
													<< "0- Back" << endl
													<< "Which agent ? ")
		&& (!(cin >> choice_) || !(choice_>=0 && choice_<=3)))
			cleanInput();
		if(choice_==0){return;} // Back

		// Activate right agents
		switch(choice_)
		{
			case 1:
				if(choice_endless==1)
				{
					endless_agent1_on_ = true;
					endless_agent2_on_ = true;
				}
				else
				{
					endless_agent1_on_ = false;
					endless_agent2_on_ = false;
				}
				break;

			case 2:
				if(choice_endless==1)
					endless_agent1_on_ = true;
				else
					endless_agent1_on_ = false;
				break;

			case 3:
				if(choice_endless==1)
					endless_agent2_on_ = true;
				else
					endless_agent2_on_ = false;
				break;
		}
	}
}

void Boss::askSetAttitude()
{
	// Ask which agent
	do {
	while(ros::ok() && this->showHumanAgents()
									&& (cout 	<< "0- Back" << endl
														<< "Which agent ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=int(agent_managers_.size()))))
		cleanInput();
	if(choice_==0){return;} // Back
	} while(agent_managers_[choice_-1]->getType()!=HUMAN && (cout << "Not human ..." << endl << endl));
	cout << endl;
	int choice_agent = choice_-1;

	// Ask which attitude to set
	while(ros::ok() && (cout 	<< "1- NONE" << endl
														<< "2- NON_STOP" << endl
														<< "3- RANDOM" << endl
														<< "4- STOP_LOOK" << endl
														<< "5- HARASS" << endl
														<< "0- Back" << endl
														<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=5)))
		cleanInput();
	if(choice_==0){return;} // Back

	// Set attitude
	dynamic_cast<HumanManager*>(agent_managers_[choice_agent])->setAttitude(choice_-1);
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

bool Boss::showHumanAgents()
{
	for(int i=0	; i<int(agent_managers_.size()); i++)
	{
		if(agent_managers_[i]->getType()==HUMAN)
		{
			cout << i+1 << "- " << agent_managers_[i]->getName() << endl;
		}
	}

	return true;
}

void Boss::wait(float delay)
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

void Boss::threadPubEndlessAgent1()
{
	ros::Rate loop(5);
	while(ros::ok())
	{
		if(endless_agent1_on_ && int(agent_managers_.size()-1)>endless_agent1_)
		{
			if(agent_managers_[endless_agent1_]->isGoalDone())
			{
				endless_agent1_i_ = (endless_agent1_i_ + 1)%endless_goals_agent1_.size();
				agent_managers_[endless_agent1_]->publishGoal(endless_goals_agent1_[endless_agent1_i_]);
				endless_delay_.sleep();
			}
		}
		loop.sleep();
	}
}

void Boss::threadPubEndlessAgent2()
{
	ros::Rate loop(5);
	while(ros::ok())
	{
		if(endless_agent2_on_ && int(agent_managers_.size())>endless_agent2_)
		{
			if(agent_managers_[endless_agent2_]->isGoalDone())
			{
				endless_agent2_i_ = (endless_agent2_i_ + 1)%endless_goals_agent2_.size();
				agent_managers_[endless_agent2_]->publishGoal(endless_goals_agent2_[endless_agent2_i_]);
				endless_delay_.sleep();
			}
		}
		loop.sleep();
	}
}

/////////////////////////////////////////////////////////

void threadSpin()
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boss");

	// Spawn thread ros spin
	boost::thread thread_spin(threadSpin);

	Boss boss;

	// Spawn thread publish endless scenario goals
	boss.spawnThreadEndless();

	// Add agents
	HumanManager human_manager1("human");
	boss.appendAgent(&human_manager1);
	RobotManager robot_manager1("robot", "/move_base_simple/goal", "/move_base/status");
	boss.appendAgent(&robot_manager1);

	while(ros::ok())
	{
		boss.showState();
		boss.askChoice();
	}
}

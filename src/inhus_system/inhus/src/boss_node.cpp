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
	pub_goal_ =	nh_.advertise<inhus::Goal>(topic_goal, 1);

	string topic_attitude = "/boss/" + name_ + "/set_attitude";
	pub_attitude_ = nh_.advertise<std_msgs::Int32>(topic_attitude, 1);

	string topic_manual = "/boss/" + name_ + "/teleoperation";
	pub_manual_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_manual, 1);

	string topic_goal_done = "/" + name_ + "/goal_done";
	sub_goal_done_ = 	nh_.subscribe(topic_goal_done, 1, &HumanManager::goalDoneCB, this);

	string topic_goal_start = "/" + name_ + "/new_goal";
	sub_goal_start_ =	nh_.subscribe(topic_goal_start, 1, &HumanManager::goalStartCB, this);

	goal_done_ = true;
	current_attitude_ = 0;
}

void HumanManager::publishGoal(inhus::Goal goal)
{
	pub_goal_.publish(goal);
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

void HumanManager::goalDoneCB(const inhus::Goal::ConstPtr& msg)
{
	goal_done_ = true;
}

void HumanManager::goalStartCB(const inhus::Goal::ConstPtr& msg)
{
	goal_done_ = false;
}

//////////////////// ROBOT MANAGER //////////////////////

RobotManager::RobotManager(string name) : AgentManager(name)
{
	type_ = ROBOT;

	string topic_goal = "/" + name_ + "/move_base_goal";
	pub_goal_ =	nh_.advertise<geometry_msgs::PoseStamped>(topic_goal, 1);

	string topic_goal_status = "/" + name_ + "/move_base_goal_status";
	sub_goal_status_ = nh_.subscribe(topic_goal_status, 1, &RobotManager::goalStatusCB, this);

	goal_done_ = true;
}

void RobotManager::publishGoal(inhus::Goal goal)
{
	if(goal.type=="pose_goal")
	{
		inhus::Goal randomized_goal;
		randomized_goal.type = "pose_goal";
		randomized_goal.pose_goal = computeGoalWithRadius(goal.pose_goal);
		pub_goal_.publish(this->getPose(randomized_goal));
	}
	else if(goal.type=="named_goal")
		ROS_ERROR("Boss::RobotManager::PublishGoal: named_goal type isn't supported for robot!");
	else
		ROS_ERROR("Boss::RobotManager::PublishGoal: goal type unknown!");
}

void RobotManager::showState()
{
}

geometry_msgs::PoseStamped RobotManager::getPose(inhus::Goal goal)
{
	if(goal.type != "pose_goal")
		ROS_ERROR("Trying to send to the robot another goal type than \"pose_goal\"");

	geometry_msgs::PoseStamped pose;

	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";

	pose.pose.position.x = goal.pose_goal.pose.x;
	pose.pose.position.y = goal.pose_goal.pose.y;
	tf2::Quaternion q;
	q.setRPY(0,0,goal.pose_goal.pose.theta);
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
	// Ros Params
	ros::NodeHandle private_nh("~");
	private_nh.param(string("map_name"), map_name_, string("laas_adream"));
	ROS_INFO("Boss: => Params Boss :");
	ROS_INFO("map_name=%s", map_name_.c_str());


	// Init goals file name and path
	goal_file_name_ = "goals.xml";
	string goal_file_path = ros::package::getPath("inhus_navigation") + "/maps/" + map_name_ + "/" + goal_file_name_;
	// Loading the xml file
	doc_ = new TiXmlDocument(goal_file_path);
	if(!doc_->LoadFile())
		ROS_ERROR("Failed to load %s. Error : %s", goal_file_path.c_str(), doc_->ErrorDesc());
	else
		ROS_INFO("Goals file loaded");
	// Check if file is corresponding with map_name
	TiXmlHandle docHandle(doc_);
	TiXmlElement* l_map = docHandle.FirstChild("map_name").ToElement();
	string map_name_read = "";
	if(NULL != l_map->Attribute("name"))
		map_name_read = l_map->Attribute("name");
	if(map_name_read != map_name_)
		ROS_ERROR("Goals file mismatches the map_name");
	else
		ROS_INFO("Goals file corresponds with map_name");
	// Extract goals
	this->readGoalsFromXML();
	ROS_INFO("Goals extracted");
	// Show goals extracted
	this->showGoals();

	// Init
	endless_agent1_ = 0;
	endless_agent2_ = 0;
	endless_agent1_on_ = false;
	endless_agent2_on_ = false;
	endless_agent1_i_ = endless_goals_[0].size()-1;
	endless_agent2_i_ = endless_goals_[1].size()-1;

	ROS_INFO("Init done");
}

Boss::~Boss()
{
	delete doc_;
}

void Boss::readGoalsFromXML()
{
	TiXmlHandle docHandle(doc_);
	inhus::Goal goal;

	// Extracting pose_goals
	TiXmlElement* l_goal = docHandle.FirstChild("goals").FirstChild("pose_goals").FirstChild("pose_goal").ToElement();
	while(l_goal)
	{
		// Set type
		goal.type = "pose_goal";
		// Set coordinates
		if(NULL != l_goal->Attribute("x"))
			goal.pose_goal.pose.x = stof(l_goal->Attribute("x"));
		if(NULL != l_goal->Attribute("y"))
			goal.pose_goal.pose.y = stof(l_goal->Attribute("y"));
		if(NULL != l_goal->Attribute("theta"))
			goal.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
		if(NULL != l_goal->Attribute("radius"))
			goal.pose_goal.radius = stof(l_goal->Attribute("radius"));
		// Add goal
		pose_goals_.push_back(goal);

		// Move to next goal
		l_goal = l_goal->NextSiblingElement();
	}

	// Extracting named_goals
	TiXmlElement* l_named_goal = docHandle.FirstChild("goals").FirstChild("named_goals").FirstChild().ToElement();
	while(l_named_goal)
	{
		// Set type
		goal.type = "named_goal";
		// Set Name
		goal.named_goal.name = l_named_goal->Value();
		// Add goal
		named_goals_.push_back(goal);

		// Move to next goal
		l_named_goal = l_named_goal->NextSiblingElement();
	}

	// Extracting the scenarios
	TiXmlElement* l_scenario = docHandle.FirstChild("goals").FirstChild("scenarios").FirstChild().ToElement();
	while(l_scenario)
	{
		Scenario scenario;

		// Get Name
		scenario.name = l_scenario->Value();

		// Extract goals
		TiXmlElement *l_goal = l_scenario->FirstChildElement();
		while(l_goal)
		{
			string type;
			string which = l_goal->Value();

			// Get type
			if(NULL != l_goal->Attribute("type"))
				type = l_goal->Attribute("type");

			// Init goals must be positions (pose_goal)
			if(which == "init_h" && type == "pose_goal")
			{
				scenario.init_h.type = type;
				if(NULL != l_goal->Attribute("x"))
					scenario.init_h.pose_goal.pose.x = stof(l_goal->Attribute("x"));
				if(NULL != l_goal->Attribute("y"))
					scenario.init_h.pose_goal.pose.y = stof(l_goal->Attribute("y"));
				if(NULL != l_goal->Attribute("theta"))
					scenario.init_h.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
				if(NULL != l_goal->Attribute("radius"))
					scenario.init_h.pose_goal.radius = stof(l_goal->Attribute("radius"));
			}
			else if(which == "init_r" && type == "pose_goal")
			{
				scenario.init_r.type = type;
				if(NULL != l_goal->Attribute("x"))
					scenario.init_r.pose_goal.pose.x = stof(l_goal->Attribute("x"));
				if(NULL != l_goal->Attribute("y"))
					scenario.init_r.pose_goal.pose.y = stof(l_goal->Attribute("y"));
				if(NULL != l_goal->Attribute("theta"))
					scenario.init_r.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
				if(NULL != l_goal->Attribute("radius"))
					scenario.init_r.pose_goal.radius = stof(l_goal->Attribute("radius"));
			}
			else if(which == "goal_h")
			{
				scenario.goal_h.type = type;
				if(type == "named_goal")
					scenario.goal_h.named_goal.name = l_goal->Attribute("name");
				else if(type == "pose_goal")
					if(NULL != l_goal->Attribute("x"))
						scenario.goal_h.pose_goal.pose.x = stof(l_goal->Attribute("x"));
					if(NULL != l_goal->Attribute("y"))
						scenario.goal_h.pose_goal.pose.y = stof(l_goal->Attribute("y"));
					if(NULL != l_goal->Attribute("theta"))
						scenario.goal_h.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
					if(NULL != l_goal->Attribute("radius"))
						scenario.goal_h.pose_goal.radius = stof(l_goal->Attribute("radius"));
			}
			else if(which == "goal_r")
			{
				scenario.goal_r.type = type;
				if(type == "named_goal")
					scenario.goal_h.named_goal.name = l_goal->Attribute("name");
				else if(type == "pose_goal")
					if(NULL != l_goal->Attribute("x"))
						scenario.goal_r.pose_goal.pose.x = stof(l_goal->Attribute("x"));
					if(NULL != l_goal->Attribute("y"))
						scenario.goal_r.pose_goal.pose.y = stof(l_goal->Attribute("y"));
					if(NULL != l_goal->Attribute("theta"))
						scenario.goal_r.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
					if(NULL != l_goal->Attribute("radius"))
						scenario.goal_r.pose_goal.radius = stof(l_goal->Attribute("radius"));
			}

			l_goal = l_goal->NextSiblingElement();
		}
		scenarios_.push_back(scenario);

		l_scenario = l_scenario->NextSiblingElement();
	}

	// Extracting the endless mode
	TiXmlElement* l_endless_scenario = docHandle.FirstChild("goals").FirstChild("endless").FirstChild().ToElement();
	while(l_endless_scenario)
	{
		vector<inhus::Goal> endless_scenario;

		// Extract goals
		TiXmlElement *l_goal = l_endless_scenario->FirstChildElement();
		while(l_goal)
		{
			if(NULL != l_goal->Attribute("type"))
				goal.type = l_goal->Attribute("type");
			if(goal.type == "named_goal")
				goal.named_goal.name = l_goal->Value();
			else if(goal.type == "pose_goal")
			{
				if(NULL != l_goal->Attribute("x"))
					goal.pose_goal.pose.x = stof(l_goal->Attribute("x"));
				if(NULL != l_goal->Attribute("y"))
					goal.pose_goal.pose.y = stof(l_goal->Attribute("y"));
				if(NULL != l_goal->Attribute("theta"))
					goal.pose_goal.pose.theta = stof(l_goal->Attribute("theta"));
				if(NULL != l_goal->Attribute("radius"))
					goal.pose_goal.radius = stof(l_goal->Attribute("radius"));
			}
			endless_scenario.push_back(goal);

			l_goal = l_goal->NextSiblingElement();
		}

		endless_goals_.push_back(endless_scenario);
		l_endless_scenario = l_endless_scenario->NextSiblingElement();
	}
}

void Boss::showGoals()
{
	// pose_goals
	cout << "=> pose_goals <=" << endl;
	for(unsigned int i=0; i<pose_goals_.size(); i++)
		cout << "\t" << pose_goals_[i].type << " " << pose_goals_[i].pose_goal.pose.x << " " << pose_goals_[i].pose_goal.pose.y << " " << pose_goals_[i].pose_goal.pose.theta << " " << pose_goals_[i].pose_goal.radius << endl;

	// named_goals
	cout << "=> named_goals <=" << endl;
	for(unsigned int i=0; i<named_goals_.size(); i++)
		cout << "\t" << named_goals_[i].type << " " << named_goals_[i].named_goal.name << endl;

	// scenarios
	cout << "=> scenarios <=" << endl;
	for(unsigned int i=0; i<scenarios_.size(); i++)
	{
		cout << scenarios_[i].name << endl;
		cout << "\t" << "init_h:" <<  scenarios_[i].init_h.type << " " << scenarios_[i].init_h.pose_goal.pose.x << " " << scenarios_[i].init_h.pose_goal.pose.y << " " << scenarios_[i].init_h.pose_goal.pose.theta << " " << scenarios_[i].init_h.pose_goal.radius << endl;
		cout << "\t" << "init_r:" <<  scenarios_[i].init_r.type << " " << scenarios_[i].init_r.pose_goal.pose.x << " " << scenarios_[i].init_r.pose_goal.pose.y << " " << scenarios_[i].init_r.pose_goal.pose.theta << " " << scenarios_[i].init_r.pose_goal.radius << endl;
		if(scenarios_[i].goal_h.type == "named_goal")
			cout << "\t" << "goal_h:" << scenarios_[i].goal_h.type << " " << scenarios_[i].goal_h.named_goal.name << endl;
		else if(scenarios_[i].goal_h.type == "pose_goal")
			cout << "\t" << "goal_h:" <<  scenarios_[i].goal_h.type << " " << scenarios_[i].goal_h.pose_goal.pose.x << " " << scenarios_[i].goal_h.pose_goal.pose.y << " " << scenarios_[i].goal_h.pose_goal.pose.theta << " " << scenarios_[i].goal_h.pose_goal.radius << endl;
		if(scenarios_[i].goal_r.type == "named_goal")
			cout << "\t" << "goal_r:" << scenarios_[i].goal_r.type << " " << scenarios_[i].goal_r.named_goal.name << endl;
		else if(scenarios_[i].goal_r.type == "pose_goal")
			cout << "\t" << "goal_r:" <<  scenarios_[i].goal_r.type << " " << scenarios_[i].goal_r.pose_goal.pose.x << " " << scenarios_[i].goal_r.pose_goal.pose.y << " " << scenarios_[i].goal_r.pose_goal.pose.theta << " " << scenarios_[i].goal_r.pose_goal.radius << endl;
	}

	// endless
	cout << "=> endless <=" << endl;
	for(unsigned int i=0; i<endless_goals_.size(); i++)
	{
		cout << "endless (" << i << ")" << endl;
		for(unsigned int j=0; j<endless_goals_[i].size(); j++)
		{
			if(endless_goals_[i][j].type == "named_goal")
				cout << "\t" << endless_goals_[i][j].type << " " << endless_goals_[i][j].named_goal.name << endl;
			else if(endless_goals_[i][j].type == "pose_goal")
				cout << "\t" << endless_goals_[i][j].type << " " << endless_goals_[i][j].pose_goal.pose.x << " " << endless_goals_[i][j].pose_goal.pose.y << " " << endless_goals_[i][j].pose_goal.pose.theta << " " << endless_goals_[i][j].pose_goal.radius << endl;
		}
	}
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
								<< "3- Endless mode" << endl
								<< "4- Set Attitude" << endl
								<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=1 && choice_<=4)))
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

		// Enless mode
		case 3:
			this->askEndlessMode();
			break;

		// Set attitude
		case 4:
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
	while(ros::ok() && (cout	<< "1- Pose_goal" << endl
								<< "2- Named_goal" << endl
								<< "3- Enter coordinates" << endl
								<< "0- Back" << endl
								<< "Choice ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=3)))
		cleanInput();
	cout << endl;
	if(choice_==0){return;} // Back

	switch(choice_)
	{
		// From pose_goals
		case 1:{
			// Ask which goal from list to send
			while(ros::ok() && (cout 	<< "[1-" << pose_goals_.size() << "] Select a goal" << endl
										<< "0- Back" << endl
										<< "Choice ? ")
			&& (!(cin >> choice_) || !(choice_>=0 && choice_<=pose_goals_.size())))
				cleanInput();
			cout << endl;
			if(choice_==0){return;} // Back

			agent_managers_[choice_agent]->publishGoal(pose_goals_[choice_-1]);

			break;}
		
		// From named_goals
		case 2:{
			// Ask which named goal from list
			while(ros::ok() && this->showAskNamedGoal() && (cout 	<< "0- Back" << endl
																	<< "Which named goal ? ")
			&& (!(cin >> choice_) || !(choice_>=0 && choice_<=named_goals_.size())))
				cleanInput();
			cout << endl;
			if(choice_==0){return;} // Back

			agent_managers_[choice_agent]->publishGoal(named_goals_[choice_-1]);

			break;}

		// Enter coordinates
		case 3:{
			bool ok = false;
			while(!ok)
			{
				// Ask goal coordinates
				inhus::Goal goal;
				goal.type = "pose_goal";
				goal.pose_goal.radius = 0;
				while(ros::ok() && (cout 	<< "Goal x : ")
				&& (!(cin >> goal.pose_goal.pose.x)))
					cleanInput();
				while(ros::ok() && (cout 	<< "Goal y : ")
				&& (!(cin >> goal.pose_goal.pose.y)))
					cleanInput();
				while(ros::ok() && (cout 	<< "Goal theta : ")
				&& (!(cin >> goal.pose_goal.pose.theta)))
					cleanInput();

				while(ros::ok() && (cout 	<< "Send this goal : (x=" << goal.pose_goal.pose.x << ", y=" << goal.pose_goal.pose.y << ", theta=" << goal.pose_goal.pose.theta << ")" << endl
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
	while(ros::ok() && this->showAskScenarios() && (cout 	<< "0- Back" << endl
															<< "Which scenario ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=scenarios_.size())))
		cleanInput();
	cout << endl;
	if(choice_==0){return;} // Back
	int choice_scenario = choice_;

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

	// Publish goals // init_h_r goal_h_r hardcoded, bypass the agent1 and agent2
	if(delay>=0)
	{
		if(choice_init==1)
		{
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(scenarios_[choice_scenario-1].init_h);
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(scenarios_[choice_scenario-1].init_r);
		}
		else
		{
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(scenarios_[choice_scenario-1].goal_h);
			wait(delay);
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(scenarios_[choice_scenario-1].goal_r);
		}
	}
	else
	{
		if(choice_init==1)
		{
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(scenarios_[choice_scenario-1].init_h);
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(scenarios_[choice_scenario-1].init_r);
		}
		else
		{
			cout << "Publish goal : " << agent_managers_[agent2]->getName() << endl;
			agent_managers_[agent2]->publishGoal(scenarios_[choice_scenario-1].goal_r);
			wait(-delay);
			cout << "Publish goal : " << agent_managers_[agent1]->getName() << endl;
			agent_managers_[agent1]->publishGoal(scenarios_[choice_scenario-1].goal_h);
		}
	}
}

void Boss::askEndlessMode()
{
	// Set endless agents
	endless_agent1_ = 0;
	endless_agent2_ = 1;

	// Ask which agent (1, 2)
	while((cout << endl 	<< "1- " << agent_managers_[endless_agent1_]->getName() << endl
							<< "2- " << agent_managers_[endless_agent2_]->getName() << endl
							<< "0- Back" << endl
							<< "Which agent to switch ? ")
	&& (!(cin >> choice_) || !(choice_>=0 && choice_<=3)))
		cleanInput();
	if(choice_==0){return;} // Back


	switch(choice_)
	{
		case 1:
			endless_agent1_on_ = !endless_agent1_on_;
			break;

		case 2:
			endless_agent2_on_ = !endless_agent2_on_;
			break;
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

bool Boss::showAskScenarios()
{
	for(unsigned int i=0; i<scenarios_.size(); i++)
		cout << i+1 << "- " << scenarios_[i].name << endl;
	return true;
}

bool Boss::showAskNamedGoal()
{
	for(unsigned int i=0; i<named_goals_.size(); i++)
		cout << i+1 << "- " << named_goals_[i].named_goal.name << endl;
	return true;
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
				endless_agent1_i_ = (endless_agent1_i_ + 1)%endless_goals_[0].size();
				agent_managers_[endless_agent1_]->publishGoal(endless_goals_[0][endless_agent1_i_]);
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
				endless_agent2_i_ = (endless_agent2_i_ + 1)%endless_goals_[1].size();
				agent_managers_[endless_agent2_]->publishGoal(endless_goals_[1][endless_agent2_i_]);
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
	RobotManager robot_manager1("robot");
	boss.appendAgent(&robot_manager1);

	while(ros::ok())
	{
		boss.showState();
		boss.askChoice();
	}
}

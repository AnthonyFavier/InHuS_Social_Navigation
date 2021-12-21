#include "taskPlanner.h"

/////////////////// TASK PLANNER /////////////////////

TaskPlanner::TaskPlanner()
{
	ros::NodeHandle private_nh("~");
	private_nh.param(std::string("map_name"), map_name_, std::string("laas_adream"));
	ROS_INFO("TP DEBUG: map_name=%s", map_name_.c_str());

	pub_log_ =	nh_.advertise<std_msgs::String>("log", 100);

	service_ = nh_.advertiseService("compute_plan", &TaskPlanner::computePlan, this);
	printf("compute_plan service is on\n");

	// INIT GOALS
	goal_file_name_ = "goals.xml";
	std::string goal_file_path = ros::package::getPath("inhus_navigation") + "/maps/" + map_name_ + "/" + goal_file_name_;
	doc_ = new TiXmlDocument(goal_file_path);
	if(!doc_->LoadFile())
		ROS_ERROR("TP: Failed to load %s. Error : %s", goal_file_path.c_str(), doc_->ErrorDesc());
	else
		ROS_INFO("TP: Goals file loaded");
	// Check if file is corresponding with map_name
	TiXmlHandle docHandle(doc_);
	TiXmlElement* l_map = docHandle.FirstChild("map_name").ToElement();
	std::string map_name_read = "";
	if(NULL != l_map->Attribute("name"))
		map_name_read = l_map->Attribute("name");
	ROS_INFO("TP: map_name_read=%s", map_name_read.c_str());
	if(map_name_read != map_name_)
		ROS_ERROR("TP: Goals file mismatches the map_name");
	else
		ROS_INFO("TP: Goals file corresponds with map_name");
	// Extract goals
	this->readGoalsFromXML();
	ROS_INFO("TP: Goals extracted");
	// Show goals extracted
	this->showGoals();
}

bool TaskPlanner::computePlan(inhus::ComputePlan::Request& req, inhus::ComputePlan::Response& res)
{
	inhus::Action action;

	if(req.goal.type == "pose_goal")
	{
		action.type = "nav_action";
		action.state = STATE_PLANNED;

		action.nav_action.pose = req.goal.pose_goal.pose;
		action.nav_action.radius = req.goal.pose_goal.radius;
		res.actions.push_back(action);
	}
	else if(req.goal.type == "named_goal")
	{
		// using name and goals extracted from xml, fill actions

		for(unsigned int i=0; i<named_goals_.size(); i++)
		{
			if(named_goals_[i].name == req.goal.named_goal.name)
			{
				for(unsigned int j=0; j<named_goals_[i].actions.size(); j++)
					res.actions.push_back(named_goals_[i].actions[j]);
				break;
			}
		}
	}

	return true;
}

void TaskPlanner::readGoalsFromXML()
{
	ROS_INFO("TP: Start extracting from XML");
	TiXmlHandle docHandle(doc_);
	inhus::Goal goal;

	// Extracting named_goals
	TiXmlElement* l_named_goal = docHandle.FirstChild("goals").FirstChild("named_goals").FirstChild().ToElement();
	while(l_named_goal)
	{
		inhus::NamedGoal named_goal;
		named_goal.name = l_named_goal->Value();

		TiXmlElement* l_action = l_named_goal->FirstChildElement();
		while(l_action)
		{
			string value = l_action->Value();
			inhus::Action action;
			action.state = STATE_PLANNED;

			if(NULL != l_action->Attribute("type"))
				action.type = l_action->Attribute("type");
			if(action.type == "wait_action")
			{
				if(NULL != l_action->Attribute("duration"))
					action.wait_action.duration = stof(l_action->Attribute("duration"));
			}
			else if(action.type == "nav_action")
			{
				if(NULL != l_action->Attribute("x"))
					action.nav_action.pose.x = stof(l_action->Attribute("x"));
				if(NULL != l_action->Attribute("y"))
					action.nav_action.pose.y = stof(l_action->Attribute("y"));
				if(NULL != l_action->Attribute("theta"))
					action.nav_action.pose.theta = stof(l_action->Attribute("theta"));
				if(NULL != l_action->Attribute("radius"))
					action.nav_action.radius = stof(l_action->Attribute("radius"));
			}
			named_goal.actions.push_back(action);

			l_action = l_action->NextSiblingElement();			
		}
		named_goals_.push_back(named_goal);

		l_named_goal = l_named_goal->NextSiblingElement();
	}
}

void TaskPlanner::showGoals()
{
	ROS_INFO("TP: ShowGoals");
	for(unsigned int i=0; i<named_goals_.size(); i++)
	{
		ROS_INFO("TP: %s", named_goals_[i].name.c_str());
		for(unsigned int j=0; j<named_goals_[i].actions.size(); j++)
		{
			if(named_goals_[i].actions[j].type=="wait_action")
				ROS_INFO("TP:\ttype=%s duration=%f", named_goals_[i].actions[j].type.c_str(), named_goals_[i].actions[j].wait_action.duration);
			else if(named_goals_[i].actions[j].type=="nav_action")
				ROS_INFO("TP:\ttype=%s (%f,%f,%f) radius=%f", named_goals_[i].actions[j].type.c_str(), named_goals_[i].actions[j].nav_action.pose.x, named_goals_[i].actions[j].nav_action.pose.y, named_goals_[i].actions[j].nav_action.pose.theta, named_goals_[i].actions[j].nav_action.radius);
		}
	}
}

/////////////////////// MAIN /////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "taskPlanner");

	TaskPlanner task_planner;

	ros::spin();

	return 0;
}

//////////////////////////////////////////////////////

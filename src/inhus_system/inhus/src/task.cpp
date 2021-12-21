#include "task.h"

//////////////////////////////// PLAN /////////////////////////////////

Plan::Plan()
{
	state_=STATE_DONE;
	curr_action_=action_series_.begin();
}

void Plan::addAction(inhus::Action action)
{
	action_series_.push_back(action);
}

void Plan::show()
{
	ROS_INFO("Plan: size=%d", (int)action_series_.size());
	for(int i=0; i<action_series_.size(); i++)
	{
		if(action_series_[i].type == "nav_action")
			ROS_INFO("\t- Action %d: type=%s, pose=(%.2f, %.2f, %.2f) radius=%.2f state=%d", i, action_series_[i].type.c_str() , action_series_[i].nav_action.pose.x, action_series_[i].nav_action.pose.y, action_series_[i].nav_action.pose.theta, action_series_[i].nav_action.radius, action_series_[i].state);
		if(action_series_[i].type == "wait_action")
			ROS_INFO("\t- Action %d: type=%s, duration=%.2f state=%d", i, action_series_[i].type.c_str(), action_series_[i].wait_action.duration, action_series_[i].state);
	}
}

void Plan::clear()
{
	action_series_.clear();
	curr_action_=action_series_.begin();

	state_=STATE_DONE;
}

void Plan::updateState()
{
	if(action_series_.empty()
	|| action_series_.back().state==STATE_DONE)
		state_=STATE_DONE;
	else
		state_=STATE_PROGRESS;
}

bool Plan::isDone()
{
	return state_==STATE_DONE;
}

void Plan::updateCurrentAction()
{
	curr_action_=action_series_.begin();

	while((*curr_action_).state==STATE_DONE && curr_action_!=action_series_.end())
		curr_action_++;
}

std::vector<inhus::Action>::iterator Plan::getCurrentAction()
{
	return curr_action_;
}

///////////////////////////////////////////////////////////////////////

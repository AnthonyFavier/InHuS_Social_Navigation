#include "task.h"

//////////////////////////////// PLAN /////////////////////////////////

Plan::Plan()
{
	state_=UNKNOWN;
	curr_action_=action_series_.begin();
}

void Plan::addAction(Action action)
{
	action_series_.push_back(action);
}

void Plan::show()
{
	ROS_INFO("Plan: size=%d", (int)action_series_.size());
	for(int i=0; i<action_series_.size(); i++)
		ROS_INFO("\tpose= (%.2f, %.2f) state=%d", action_series_[i].action.target_pose.pose.position.x, action_series_[i].action.target_pose.pose.position.y, action_series_[i].state);
}

void Plan::clear()
{
	action_series_.clear();
	curr_action_=action_series_.begin();

	state_=UNKNOWN;
}

void Plan::updateState()
{
	if(action_series_.back().state==DONE)
		state_=DONE;
	else
		state_=PROGRESS;
}

bool Plan::isDone()
{
	return state_==DONE;
}

void Plan::updateCurrentAction()
{
	curr_action_=action_series_.begin();

	while((*curr_action_).state==DONE && curr_action_!=action_series_.end())
		curr_action_++;
}

std::vector<Action>::iterator Plan::getCurrentAction()
{
	return curr_action_;
}

///////////////////////////////////////////////////////////////////////

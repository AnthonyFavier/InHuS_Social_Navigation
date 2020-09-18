#include "task.h"

//////////////////////////////// PLAN /////////////////////////////////

Plan::Plan()
{
	state_=UNKNOWN;
	curr_action_=action_series_.begin();
}

void Plan::addAction(human_sim::Action action)
{
	Action act;
	act.action = action;
	act.state=PLANNED;
	action_series_.push_back(action);
}

void Plan::show()
{
	printf("Plan: size=%d\n", action_series_.size());
	for(int i=0; i<action_series_.size(); i++)
		printf("\ttype=%s pose= (%f, %f, %f)", actions_series_[i].action.type, actions_series_[i].action.pose.x, actions_series_[i].action.pose.y, actions_series_[i].action.pose.theta);
}

void Plan::clear()
{
	action_series_.clear();
	current_action_=0;

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

	while(*curr_action_.state==DONE && curr_action_!=action_series_.end())
		curr_action_++;
}

std::vector<Action>::iterator Plan::getCurrentAction()
{
	return *curr_action_;
}

///////////////////////////////////////////////////////////////////////

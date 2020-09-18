#include "task.h"

///////////////////////////// ACTION //////////////////////////////////

Action::Action()
{
	state_=UNKNOWN;
}

Action::~Action(){}

void Action::setState(State state)
{
	state_=state;
}

State Action::getState()
{
	return state_;
}

///////////////////////// TYPE OF ACTION //////////////////////////////

Movement::Movement(float x, float y, float theta): Action()
{
	type="Movement";

	destination_.x = x;
	destination_.y = y;
	destination_.theta = theta;
}

void Movement::show()
{
	std::cout << type << " (" << destination_.x << ", " << destination_.y << ", " << destination_.theta << ") " << state_ << std::endl;
}

Pose2D Movement::getDestination()
{
	return destination_;
}

/*PickPlace::PickPlace(): Action()
{
	type="PickPlace";
}*/

//////////////////////////////// PLAN /////////////////////////////////

Plan::Plan()
{
	state_=UNKNOWN;
	current_action_=0;
}

void Plan::addAction(Action* action)
{
	action_series_.push_back(action);
}

void Plan::show()
{
	for(int i=0; i<action_series_.size(); i++)
		action_series_[i]->show();
}

void Plan::clear()
{
	for(int i=0; i<action_series_.size(); i++)
		delete action_series_[i];
	action_series_.clear();
	current_action_=0;

	state_=UNKNOWN;
}

void Plan::setState(State state)
{
	state_=state;
}

bool Plan::isDone()
{
	if(action_series_.back()->getState()==DONE)
		state_=DONE;
	else
		state_=PROGRESS;

	return state_==DONE;
}

void Plan::updateCurrentAction()
{
	for(int i=0; i<action_series_.size(); i++)
	{
		if(action_series_[i]->getState()!=DONE)
		{
			current_action_=i;
			break;
		}
	}
}

State Plan::getCurrentActionState()
{
	return action_series_[current_action_]->getState();
}

void Plan::setCurrentActionState(State state)
{
	action_series_[current_action_]->setState(state);
}

Pose2D Plan::getCurrentActionDestination()
{
	Movement* mvt = reinterpret_cast<Movement*>(action_series_[current_action_]);
	return mvt->getDestination();
}

///////////////////////////////////////////////////////////////////////

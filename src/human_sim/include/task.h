#ifndef DEF_TASK
#define DEF_TASK

#include "ros/ros.h"
#include <vector>
#include "type.h"
#include <move_base_msgs/MoveBaseAction.h>

enum State{UNKNOWN=0, PLANNED, NEEDED, READY, PROGRESS, DONE, FAILED};

/////////////////// ACTION //////////////////////

struct Action
{
	move_base_msgs::MoveBaseGoal action;
	State state;
};

////////////////////// PLAN /////////////////////

class Plan
{
public:
	Plan();

	void addAction(Action action);

	void show();
	void clear();

	void setState(State state);

	bool isDone();

	void updateCurrentAction();
	std::vector<Action>::iterator getCurrentAction();
	void updateState();

private:
	State state_;
	std::vector<Action> action_series_;
	std::vector<Action>::iterator curr_action_;
};

/////////////////////////////////////////////////

#endif

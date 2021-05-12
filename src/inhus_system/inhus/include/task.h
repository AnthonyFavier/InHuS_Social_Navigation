#ifndef DEF_TASK
#define DEF_TASK

#include "ros/ros.h"
#include <vector>
#include "inhus/Action.h"

#define STATE_UNKNOWN 0
#define STATE_PLANNED 1
#define STATE_NEEDED 2
#define STATE_READY 3
#define STATE_PROGRESS 4
#define STATE_DONE 5
#define STATE_FAILED 6

////////////////////// PLAN /////////////////////

class Plan
{
public:
	Plan();

	void addAction(inhus::Action action);

	void show();
	void clear();

	bool isDone();

	void updateCurrentAction();
	std::vector<inhus::Action>::iterator getCurrentAction();
	void updateState();

private:
	int state_;
	std::vector<inhus::Action> action_series_;
	std::vector<inhus::Action>::iterator curr_action_;
};

/////////////////////////////////////////////////

#endif

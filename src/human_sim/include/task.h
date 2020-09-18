#ifndef DEF_TASK
#define DEF_TASK

#include <iostream>
#include <vector>

struct Pose2D
{
	float x;
	float y;
	float theta;
};

enum State{UNKNOWN=0, PLANNED, NEEDED, READY, PROGRESS, DONE, FAILED};

/////////////////// ACTION //////////////////////

class Action
{
public:
	Action();
	virtual ~Action()=0;
	virtual void show()=0;
	void setState(State state);
	State getState();
protected:
	State state_;

	std::string type;
	std::vector<std::string> preconditions_;
	//std::vector<std::string> postconditions_;
};

////////////////// TYPE OF ACTION ///////////////

class Movement: public Action
{
public:
	Movement(float x, float y, float theta);
	~Movement(){};

	void show();
	Pose2D getDestination();
private:
	Pose2D destination_;
};

/*class PickPlace: public Action
{
public:
	PickPlace();
	~PickPlace(){};

	void show();
private:
}*/

////////////////////// PLAN /////////////////////

class Plan
{
public:
	Plan();
	void addAction(Action* action);

	void show();
	void clear();

	void setState(State state);

	void updateCurrentAction();
	bool isDone();

	int current_action_;
	State getCurrentActionState();
	void setCurrentActionState(State state);

	Pose2D getCurrentActionDestination();
private:
	State state_;
	std::vector<Action*> action_series_;
};

/////////////////////////////////////////////////

#endif

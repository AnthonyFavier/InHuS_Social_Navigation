#ifndef TYPES
#define TYPES

#include "inhus/Goal.h"

#define PI 3.1415926535897932384626433832795

struct GoalArea
{
	inhus::Goal goal;
	float radius;
};

GoalArea computeGoalWithRadius(GoalArea goal)
{
	if(goal.radius > 0)
	{
		float alpha = (rand()%(2*314))/100 - PI;
		float r = (rand()%(int(goal.radius*100)))/100.0;

		goal.goal.x = goal.goal.x + r * cos(alpha);
		goal.goal.y = goal.goal.y + r * sin(alpha);

		goal.radius = 0;
	}
	return goal;
}

#endif

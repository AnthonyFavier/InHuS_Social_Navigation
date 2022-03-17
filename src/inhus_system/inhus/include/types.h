#ifndef TYPES
#define TYPES

#include "inhus/Goal.h"

#define PI 3.1415926535897932384626433832795

inhus::PoseGoal computeGoalWithRadius(inhus::PoseGoal goal)
{
	inhus::PoseGoal new_goal = goal;

	if(goal.radius > 0)
	{
		float alpha = (rand()%(2*314))/100 - PI;
		float r = (rand()%(int(goal.radius*100)))/100.0;

		new_goal.pose.x = goal.pose.x + r * cos(alpha);
		new_goal.pose.y = goal.pose.y + r * sin(alpha);
	}
	return new_goal;
}

inhus::Action computeActionWithRadius(inhus::Action action)
{
	inhus::Action rand_action = action;

	if(action.type == "nav_action")
	{
		if(action.nav_action.radius > 0)
		{
			float alpha = (rand()%(2*314))/100 - PI;
			float r = (rand()%(int(action.nav_action.radius*100)))/100.0;

			rand_action.nav_action.pose.x += r * cos(alpha);
			rand_action.nav_action.pose.y += r * sin(alpha);
		}
	}
	return rand_action;
}

#endif

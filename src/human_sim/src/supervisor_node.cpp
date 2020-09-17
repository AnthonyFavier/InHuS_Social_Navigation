#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(ros::NodeHandle nh)
{
	nh_ = nh;

	sub_new_goal_ = nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);

	state_global_ = GET_GOAL;
	choice_goal_decision_ = SPECIFIED;
	new_goal_ = false;
}

void Supervisor::FSM()
{
	switch(state_global_)
	{
		case GET_GOAL:
			ROS_INFO("GET_GOAL");
			switch(choice_goal_decision_)
			{
				case AUTONOMOUS:
					// Decide a goal
					this->findAGoal();
					state_global_ = ASK_PLAN;
					break;

				case SPECIFIED:
					// Wait for the boss
					if(new_goal_)
					{
						state_global_ = ASK_PLAN;
						new_goal_ = false;
					}
					break;
			}
			break;

		case ASK_PLAN:
			ROS_INFO("ASK_PLAN");
			state_global_ = EXEC_PLAN;
			break;

		case EXEC_PLAN:
			ROS_INFO("EXEC_PLAN");
			state_global_ = GET_GOAL;
			break;

		default:
			state_global_=GET_GOAL;
			break;
	}
}

void Supervisor::findAGoal()
{

}

void Supervisor::newGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ROS_INFO("New goal received !");

	new_goal_=true;

	current_goal_.type="Position";
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

/////////////////////////////// MAIN /////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);

	Supervisor supervisor(nh);

	while(ros::ok())
	{
		supervisor.FSM();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////

#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(ros::NodeHandle nh): plan_(), client("do_action", true)
{
	nh_ = nh;

	sub_new_goal_ = nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_human_pos_ = nh_.subscribe("human_pos", 100, &Supervisor::humanPosCallback, this);

	state_global_ = GET_GOAL;
	choice_goal_decision_ = AUTONOMOUS;

	goal_received_ = false;

	human_pos_.x=0;
	human_pos_.y=0;
	human_pos_.theta=0;

	srand(time(NULL));

	client.waitForServer();
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
					if(goal_received_)
					{
						goal_received_ = false;
						state_global_ = ASK_PLAN;
					}
					break;
			}
			break;

		case ASK_PLAN:
			ROS_INFO("ASK_PLAN");
			this->askPlan();
			plan_.show();
			state_global_ = EXEC_PLAN;
			break;

		case EXEC_PLAN:
			ROS_INFO("EXEC_PLAN");
			plan_.show();
			if(!plan_.isDone())
			{
				// check current action
				plan_.updateCurrentAction();
				std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

				// if PLANNED or NEEDED
				// 	check precond
				// 	if ok -> READY
				// 	else -> NEEDED
				//
				// else if current action READY
				// 	do action (send to geometric planner)
				// 	action -> progress
				//
				// else if PROGRESS
				// 	check postcondition
				// 	if ok -> DONE

				if((*curr_action).state==PLANNED || (*curr_action).state==NEEDED)
				{
					// check preconditions
					// => for now no checking

					//if(precond==ok)
						(*curr_action).state=READY;
					//else
					//	(*curr_action).state=NEEDED;
				}
				else if((*curr_action).state==READY)
				{
					// send to geometric planner (do action)
					// client action
					(*curr_action).state=PROGRESS;
				}
				else if((*curr_action).state==PROGRESS)
				{
					// check postconditions
					// for now : if human at destination

					/*std::cout << "human_pos = " << human_pos_.x << "," << human_pos_.y << "," << human_pos_.theta << std::endl;
					Pose2D dest = plan_.getCurrentActionDestination();
					if(human_pos_.x==dest.x && human_pos_.y==dest.y)
						plan_.setCurrentActionState(DONE);*/

					// client wait for result
					// *curr_action_.state=DONE;
				}
			}
			else
			{
				ROS_INFO("Plan is DONE !");
				state_global_ = GET_GOAL;
			}
			break;

		default:
			state_global_=GET_GOAL;
			break;
	}
}

void Supervisor::findAGoal()
{
	current_goal_.type="Position";
	current_goal_.x=(rand()%100)/10.0;
	current_goal_.y=(rand()%100)/10.0;
	current_goal_.theta=(rand()%30)/10.0;
}

void Supervisor::askPlan()
{
	plan_.clear();

	//symPlanner.askPlan(&plan);
	
	// trickery for only navigation without sub goal
	Action action;
	action.action.type="movement";
	action.state=PLANNED;
	plan_.addAction(action);
}

void Supervisor::newGoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ROS_INFO("New goal received !");

	goal_received_=true;

	current_goal_.type="Position";
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

void Supervisor::humanPosCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	ROS_INFO("HumanPos received !");

	human_pos_.x=msg->x;
	human_pos_.y=msg->y;
	human_pos_.theta=msg->theta;
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

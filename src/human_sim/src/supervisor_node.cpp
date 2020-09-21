#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(ros::NodeHandle nh): plan_(), client_action_("do_action", true)
{
	///////////////////////////////////
	choice_goal_decision_ = SPECIFIED; // AUTONOMOUS or SPECIFIED
	///////////////////////////////////

	nh_ = nh;

	client_plan_ = nh_.serviceClient<human_sim::ComputePlan>("compute_plan");

	sub_new_goal_  = 		nh_.subscribe("boss/new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_teleoperation_boss_ =	nh_.subscribe("boss/teleoperation", 100, &Supervisor::teleopBossCallback, this);
	sub_operating_mode_ =		nh_.subscribe("boss/operationg_mode", 100, &Supervisor::operatingModeBossCallback, this);

	state_global_ = GET_GOAL;

	goal_received_ = false;

	srand(time(NULL));

	printf("Waiting for action server\n");
	client_action_.waitForServer();
	printf("Connected to action server\n");
}

void Supervisor::FSM()
{
	printf("\n");
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

				switch((*curr_action).state)
				{
					case PLANNED:
					case NEEDED:
						printf("NEEDED\n");
						// check preconditions
						// => for now no checking

						//if(precond==ok)
							(*curr_action).state=READY;
						//else
						//	(*curr_action).state=NEEDED;
						break;

					case READY:
						printf("READY\n");
						// send to geometric planner (do action)
						client_action_.sendGoal((*curr_action).action);
						(*curr_action).state=PROGRESS;
						break;

					case PROGRESS:
						printf("PROGRESS\n");
						// check postconditions
						// for now : if human at destination
						// done in geoPlanner ?

						if(client_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
						{
							printf("Client succeeded\n");
							(*curr_action).state = DONE;
						}
						break;
				}
			}
			else
			{
				ROS_INFO("Plan is DONE !");
				plan_.clear();
				state_global_ = GET_GOAL;
			}
			plan_.updateState();
			break;

		default:
			state_global_=GET_GOAL;
			break;
	}
}

void Supervisor::findAGoal()
{
	current_goal_.type = 	"Position";
	current_goal_.x = 	(rand()%100)/10.0;
	current_goal_.y = 	(rand()%100)/10.0;
	current_goal_.theta = 	(rand()%30)/10.0;
}

void Supervisor::askPlan()
{
	plan_.clear();

	//symPlanner.askPlan(&plan);
	human_sim::ComputePlan srv;
	srv.request.goal.type = 	current_goal_.type;
	srv.request.goal.x = 		current_goal_.x;
	srv.request.goal.y = 		current_goal_.y;
	srv.request.goal.theta = 	current_goal_.theta;
	if(!client_plan_.call(srv))
	{
		ROS_ERROR("Ask for a plan failed");
		exit(-1);
	}
	
	Action action;
	for(int i=0; i<srv.response.actions.size(); i++)
	{
		action.action.type = 			srv.response.actions[i].type;
		action.action.destination.x = 		srv.response.actions[i].destination.x;
		action.action.destination.y = 		srv.response.actions[i].destination.y;
		action.action.destination.theta =	srv.response.actions[i].destination.theta;
		action.state=PLANNED;
		plan_.addAction(action);
	}
}

void Supervisor::newGoalCallback(const human_sim::GoalConstPtr& msg)
{
	ROS_INFO("New goal received!");

	goal_received_=true;

	current_goal_.type=msg->type;
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

void Supervisor::teleopBossCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
}

void Supervisor::operatingModeBossCallback(const std_msgs::Int32::ConstPtr& msg)
{
	switch(msg->data)
	{
		case AUTONOMOUS: // 0
			choice_goal_decision_=AUTONOMOUS;
			break;

		case SPECIFIED:  // 1
			choice_goal_decision_=SPECIFIED;
			break;

		default:
			choice_goal_decision_=AUTONOMOUS;
			break;
	}
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

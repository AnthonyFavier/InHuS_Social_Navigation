#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(ros::NodeHandle nh): plan_(), client_action_("do_action", true)
{
	///////////////////////////////////
	choice_goal_decision_ = SPECIFIED; // AUTONOMOUS or SPECIFIED
	///////////////////////////////////

	nh_ = nh;

	client_plan_ = nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_goal_ = nh_.serviceClient<human_sim::ChooseGoal>("choose_goal");

	sub_new_goal_  = 	nh_.subscribe("boss/new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_teleop_boss_ =	nh_.subscribe("boss/teleoperation", 100, &Supervisor::teleopBossCallback, this);
	sub_operating_mode_ =	nh_.subscribe("boss/operating_mode", 100, &Supervisor::operatingModeBossCallback, this);

	pub_teleop_ = 	nh_.advertise<geometry_msgs::Twist>("controller/teleop_cmd", 100);

	state_global_ = GET_GOAL;

	goal_received_ = false;

	ros::service::waitForService("compute_plan");
	printf("Connected to taskPlanner server\n");

	ros::service::waitForService("choose_goal");
	printf("Connected to choose_goal server");

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
					// Find itself a goal
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

				default:
					choice_goal_decision_ = SPECIFIED;
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
			if(goal_received_)
			{
				goal_received_ = false;
				state_global_ = ASK_PLAN;
			}
			else
			{
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
			}
			break;

		default:
			state_global_=GET_GOAL;
			break;
	}
}

void Supervisor::findAGoal()
{
	human_sim::ChooseGoal srv;
	client_goal_.call(srv);

	current_goal_.type = 	srv.response.goal.type;	
	current_goal_.x = 	srv.response.goal.x;
	current_goal_.y = 	srv.response.goal.y;
	current_goal_.theta = 	srv.response.goal.theta;
}

void Supervisor::askPlan()
{
	plan_.clear();

	human_sim::ComputePlan srv;
	srv.request.goal.type = 	current_goal_.type;
	srv.request.goal.x = 		current_goal_.x;
	srv.request.goal.y = 		current_goal_.y;
	srv.request.goal.theta = 	current_goal_.theta;
	if(!client_plan_.call(srv))
	{
		ROS_ERROR("Failure while asking for a plan");
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
	pub_teleop_.publish(*msg);
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

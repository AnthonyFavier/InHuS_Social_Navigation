#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(): plan_(), client_action_("move_base", true)
{
	///////////////////////////////////
	choice_goal_decision_ = SPECIFIED; // AUTONOMOUS or SPECIFIED
	///////////////////////////////////

	client_plan_ = nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_goal_ = nh_.serviceClient<human_sim::ChooseGoal>("choose_goal");

	sub_new_goal_  = 	nh_.subscribe("boss/new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_teleop_boss_ =	nh_.subscribe("boss/teleoperation", 100, &Supervisor::teleopBossCallback, this);
	sub_operating_mode_ =	nh_.subscribe("boss/operating_mode", 100, &Supervisor::operatingModeBossCallback, this);
	sub_cancel_goal_ = 	nh_.subscribe("move_base/cancel", 100, &Supervisor::cancelGoalCallback, this);

	pub_teleop_ = 		nh_.advertise<geometry_msgs::Twist>("controller/teleop_cmd", 100);
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);

	state_global_ = GET_GOAL;

	goal_received_ = false;

	reset_after_goal_aborted_ = false;
	goal_aborted_count_ = 0;

	ros::service::waitForService("compute_plan");
	printf("Connected to taskPlanner server\n");

	ros::service::waitForService("choose_goal");
	printf("Connected to choose_goal server\n");

	printf("Waiting for action server\n");
	client_action_.waitForServer();
	printf("Connected to action server\n");
}

void Supervisor::FSM()
{
	printf("\n");

	this->checkGoalAborted();

	// modified only in : here, cancelGoalCallback, checkGoalAborted
	switch(state_global_)
	{
		case GET_GOAL:
			ROS_INFO("GET_GOAL");
			switch(choice_goal_decision_)
			{
				case AUTONOMOUS:
					// Find itself a goal
					printf("AUTONOMOUS\n");
					this->findAGoal();
					state_global_ = ASK_PLAN;
					break;

				case SPECIFIED:
					// Wait for the boss
					printf("SPECIFIED\n");
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
			printf("current_goal : %s (%f, %f, %f)\n", current_goal_.type.c_str(), current_goal_.x, current_goal_.y, current_goal_.theta);
			reset_after_goal_aborted_=false;
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
							// send to geometric planner 
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
					pub_goal_done_.publish(current_goal_);
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

void Supervisor::checkGoalAborted()
{
	printf("CLIENT STATE : ");
	actionlib::SimpleClientGoalState state = client_action_.getState();
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("SUCCEEDED");
	else if(state == actionlib::SimpleClientGoalState::PENDING)
		printf("PENDING");
	else if(state == actionlib::SimpleClientGoalState::ACTIVE)
		printf("ACTIVE");
	else if(state == actionlib::SimpleClientGoalState::RECALLED)
		printf("RECALLED");
	else if(state == actionlib::SimpleClientGoalState::REJECTED)
		printf("REJECTED");
	else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
		printf("PREEMPTED");
	else if(state == actionlib::SimpleClientGoalState::ABORTED)
		printf("ABORTED");
	else if(state == actionlib::SimpleClientGoalState::LOST)
		printf("LOST");
	printf("\n");

	if(!reset_after_goal_aborted_ 
	&& state==actionlib::SimpleClientGoalState::ABORTED)
	{
		if(goal_aborted_count_ < 3)
			goal_aborted_count_++;
		else
		{
			state_global_ = GET_GOAL;
			goal_aborted_count_ = 0;
			reset_after_goal_aborted_ = true;
		}
	}
}

void Supervisor::findAGoal()
{
	human_sim::ChooseGoal srv;
	ros::service::waitForService("choose_goal");
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
	ros::service::waitForService("compute_plan");
	while(!client_plan_.call(srv))
	{
		ROS_ERROR("Failure while asking for a plan, asking again in 1s");
		ros::Duration(1).sleep();
	}
	
	Action ac;
	for(int i=0; i<srv.response.actions.size(); i++)
	{
		ac.action = srv.response.actions[i];
		ac.state=PLANNED;
		plan_.addAction(ac);
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

void Supervisor::cancelGoalCallback(const actionlib_msgs::GoalID::ConstPtr& msg)
{
	state_global_=GET_GOAL;
}

/////////////////////////////// MAIN /////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "supervisor");

	Supervisor supervisor;

	ros::Rate loop_rate(15);

	while(ros::ok())
	{
		supervisor.FSM();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////

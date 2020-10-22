#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor(): plan_(), client_action_("move_base", true)
{
	///////////////////////////////////
	choice_goal_decision_ = SPECIFIED; // AUTONOMOUS or SPECIFIED
	///////////////////////////////////

	client_plan_ = nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_goal_ = nh_.serviceClient<human_sim::ChooseGoal>("choose_goal");

	sub_human_pose_ = 	nh_.subscribe("human_model/human_pose", 100, &Supervisor::humanPoseCallback, this);
	sub_new_goal_  = 	nh_.subscribe("boss/new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_teleop_boss_ =	nh_.subscribe("boss/teleoperation", 100, &Supervisor::teleopBossCallback, this);
	sub_operating_mode_ =	nh_.subscribe("boss/operating_mode", 100, &Supervisor::operatingModeBossCallback, this);
	sub_path_ =		nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);

	pub_teleop_ = 		nh_.advertise<geometry_msgs::Twist>("controller/teleop_cmd", 100);
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);
	pub_cancel_goal_ = 	nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
	pub_log_ =		nh_.advertise<std_msgs::String>("log", 100);

	service_set_get_goal_ =	nh_.advertiseService("set_get_goal", &Supervisor::setGetGoal, this);

	state_global_ = GET_GOAL;

	goal_received_ = false;

	first_blocked_ = true;

	reset_after_goal_aborted_ = 	false;
	goal_aborted_count_ = 		0;
	path_diff_threshold_ = 		3;

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	first_path_received_ = true;

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

	// modified only in : here, cancelGoalCallback
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
								first_path_received_=true;
								(*curr_action).state = DONE;
							}
							break;
					}

					if(this->checkPlanFailure())
						state_global_ = BLOCKED_BY_ROBOT;
				}
				else
				{
					ROS_INFO("Plan is DONE !");
					pub_goal_done_.publish(current_goal_);
					plan_.clear();
					current_path_.poses.clear();
					previous_path_.poses.clear();
					state_global_ = GET_GOAL;
				}
				plan_.updateState();
			}
			break;

		case BLOCKED_BY_ROBOT:
			ROS_INFO("BLOCKED_BY_ROBOT");
			if(first_blocked_)
			{
				pub_cancel_goal_.publish(actionlib_msgs::GoalID());
				first_blocked_=false;
			}
			else
			{
			}
			break;

		default:
			state_global_=GET_GOAL;
			break;
	}
}

bool Supervisor::checkPlanFailure()
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

	// Check goal aborted (failure computing a plan)
	if(!reset_after_goal_aborted_ 
			&& state==actionlib::SimpleClientGoalState::ABORTED)
	{
		if(goal_aborted_count_ < 3)
			goal_aborted_count_++;
		else
		{
			goal_aborted_count_ = 0;
			reset_after_goal_aborted_ = true;

			return true;
		}
	}

	// Check if path changed too much
	printf("current_size = %d previous_size = %d\n", current_path_.poses.size(), previous_path_.poses.size());
	if(previous_path_.poses.size() != 0 && current_path_.poses.size() != 0)
	{
		if(float(current_path_.poses.size())/float(previous_path_.poses.size()) > 1.5)
			return true;



		///////////////////////////////////////////////////
		// CODE MORT POUR EVITER DE COMMENTER !!!!!!!!!! //
		///////////////////////////////////////////////////
		//						 //
		//			 | 			 //
		//			\|/			 //
		//						 //
		///////////////////////////////////////////////////
		if(false)
		{
			printf("test failure\n");
			nav_msgs::Path path1, path2; // plan1 is the longest
			if(current_path_.poses.size() > previous_path_.poses.size())
			{
				path1 = current_path_;
				path2 = previous_path_;
			}
			else
			{
				path1 = previous_path_;
				path2 = current_path_;
			}

			/*geometry_msgs::PoseStamped pose;
			  pose.pose.position.x=0;
			  pose.pose.position.y=0;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=1;
			  pose.pose.position.y=1;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=2;
			  pose.pose.position.y=2;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=3;
			  pose.pose.position.y=3;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=4;
			  pose.pose.position.y=4;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=5;
			  pose.pose.position.y=5;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=6;
			  pose.pose.position.y=6;
			  path2.poses.push_back(pose);
			  pose.pose.position.x=7;
			  pose.pose.position.y=7;
			  path2.poses.push_back(pose);

			  pose.pose.position.x=0;
			  pose.pose.position.y=0;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=1;
			  pose.pose.position.y=0;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=2.5;
			  pose.pose.position.y=0;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=4;
			  pose.pose.position.y=0;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=5;
			  pose.pose.position.y=0;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=6;
			  pose.pose.position.y=1;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=7;
			  pose.pose.position.y=2;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=8;
			  pose.pose.position.y=3;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=9;
			  pose.pose.position.y=5;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=9;
			  pose.pose.position.y=7;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=9;
			  pose.pose.position.y=8.5;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=7;
			  pose.pose.position.y=8.5;
			  path1.poses.push_back(pose);
			  pose.pose.position.x=5.5;
			  pose.pose.position.y=8;
			  path1.poses.push_back(pose);	
			  pose.pose.position.x=7;
			  pose.pose.position.y=7;
			  path1.poses.push_back(pose);*/

			float ratio = float(path1.poses.size())/float(path2.poses.size());

			printf("n_path1=%d n_path2=%d ration=%f\n", path1.poses.size(), path2.poses.size(), ratio);

			std::vector<int> corr_path2[path2.poses.size()];
			for(int i=0; i<path1.poses.size(); i++)
			{
				int index = ceil(i/ratio);
				index = index >= path2.poses.size() ? path2.poses.size()-1 : index;

				corr_path2[index].push_back(i);
			}

			std::vector<geometry_msgs::PoseStamped> path1_reduced;
			for(int i=0; i<path2.poses.size(); i++)
			{
				if(corr_path2[i].size()==1)
					path1_reduced.push_back(path1.poses[corr_path2[i][0]]);
				else
				{
					geometry_msgs::PoseStamped mean_pose;
					for(int j=0; j<corr_path2[i].size(); j++)
					{
						//printf("plan2[%d] <=> plan1[%d]\n", i, corr_path2[i][j]);
						mean_pose.pose.position.x += path1.poses[corr_path2[i][j]].pose.position.x;
						mean_pose.pose.position.y += path1.poses[corr_path2[i][j]].pose.position.y;
					}

					mean_pose.pose.position.x /= corr_path2[i].size();
					mean_pose.pose.position.y /= corr_path2[i].size();

					path1_reduced.push_back(mean_pose);
				}
			}

			/*printf("path1 reduced :\n");
			  for(int i=0; i<path1_reduced.size(); i++)
			  printf("pose[%d]= %f,%f\n", path1_reduced[i].pose.position.x, path1_reduced[i].pose.position.y);
			  */


			// Compute difference
			float diff=0;
			float dist;
			for(int i=0; i<path2.poses.size(); i++)
			{
				dist = sqrt(pow(path2.poses[i].pose.position.x-path1_reduced[i].pose.position.x,2) 
						+ pow(path2.poses[i].pose.position.y-path1_reduced[i].pose.position.y,2));

				diff += dist;
			}

			diff /= path2.poses.size();
			printf("diff=%f\n", diff);

			if(diff > path_diff_threshold_)
				return true;
		}
	}

	return false;
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

bool Supervisor::setGetGoal(human_sim::SetGetGoal::Request &req, human_sim::SetGetGoal::Response &res)
{
	state_global_=GET_GOAL;

	return true;
}

void Supervisor::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
	printf("pathCallback ! \n");

	if(first_path_received_)
	{
		printf("first !\n");
		current_path_ = *path;
		previous_path_.poses.clear();
		first_path_received_ = false;
	}
	else
	{
		printf("CUTTING path !\n");
		// seek pose closest to current_pose
		// only keep path from current_pose to the end
		if(current_path_.poses.size()!=0)
		{
			float dist = sqrt(pow(current_path_.poses[0].pose.position.x-human_pose_.x,2) + pow(current_path_.poses[0].pose.position.y-human_pose_.y,2));
			float dist_min = dist;
			int i_min = 0;
			for(int i=1; i<current_path_.poses.size(); i++)
			{
				dist = sqrt(pow(current_path_.poses[i].pose.position.x-human_pose_.x,2) + pow(current_path_.poses[i].pose.position.y-human_pose_.y,2));
				if(dist < dist_min)
				{
					dist_min = dist;
					i_min = i;
				}
			}

			previous_path_.poses.clear();
			for(int i=i_min; i<current_path_.poses.size(); i++)
				previous_path_.poses.push_back(current_path_.poses[i]);
		}
		else
			previous_path_ = current_path_;


		current_path_ = *path;
	}
}

void Supervisor::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	human_pose_.x = 	msg->x;
	human_pose_.y = 	msg->y;
	human_pose_.theta = 	msg->theta;
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

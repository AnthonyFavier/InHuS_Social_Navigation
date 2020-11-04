#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor()
: plan_()
, client_action_("move_base", true)
, dur_replan_(0.5)
, dur_replan_blocked_(0.7)
, dur_check_pose_blocked_(0.1)
{
	///////////////////////////////////
	choice_goal_decision_ = SPECIFIED; // AUTONOMOUS or SPECIFIED
	///////////////////////////////////

	client_plan_ = 			nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_goal_ = 			nh_.serviceClient<human_sim::ChooseGoal>("choose_goal");
	client_make_plan_ =		nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
	client_cancel_goal_and_stop_= 	nh_.serviceClient<human_sim::CancelGoalAndStop>("cancel_goal_and_stop");

	sub_human_pose_ = 	nh_.subscribe("human_model/human_pose", 100, &Supervisor::humanPoseCallback, this);
	sub_new_goal_  = 	nh_.subscribe("/boss/human/new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_teleop_boss_ =	nh_.subscribe("/boss/human/teleoperation", 100, &Supervisor::teleopBossCallback, this);
	sub_operating_mode_ =	nh_.subscribe("/boss/human/operating_mode", 100, &Supervisor::operatingModeBossCallback, this);
	sub_path_ =		nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);

	pub_teleop_ = 		nh_.advertise<geometry_msgs::Twist>("controller/teleop_cmd", 100);
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);
	pub_log_ =		nh_.advertise<std_msgs::String>("log", 100);
	pub_marker_rviz_ =	nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	service_set_get_goal_ =			nh_.advertiseService("set_get_goal", &Supervisor::setGetGoal, this);
	service_get_choiceGoalDecision_ = 	nh_.advertiseService("get_choiceGoalDecision", &Supervisor::getChoiceGoalDecision, this);

	marker_rviz_.header.frame_id = 		"map";
	marker_rviz_.type = 			3;
	marker_rviz_.pose.position.x = 		0;
	marker_rviz_.pose.position.y = 		0;
	marker_rviz_.pose.position.z = 		0.25;
	marker_rviz_.pose.orientation.x = 	0;
	marker_rviz_.pose.orientation.y = 	0;
	marker_rviz_.pose.orientation.z = 	0;
	marker_rviz_.pose.orientation.w = 	0;
	marker_rviz_.scale.x = 			0.1;
	marker_rviz_.scale.y = 			0.1;
	marker_rviz_.scale.z = 			0.5;
	marker_rviz_.color.r = 			1;
	marker_rviz_.color.g = 			1;
	marker_rviz_.color.b = 			0;
	marker_rviz_.color.a = 			0;

	state_global_ = GET_GOAL;

	goal_received_ = false;

	first_blocked_ = true;
	replan_success_nb_ = 0;
	first_not_feasible_ = true;

	goal_aborted_count_ = 		0;
	last_replan_ = 			ros::Time::now();

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	ros::service::waitForService("compute_plan");
	printf("Connected to taskPlanner server\n");

	ros::service::waitForService("choose_goal");
	printf("Connected to choose_goal server\n");

	ros::service::waitForService("move_base/GlobalPlanner/make_plan");
	printf("Connected to make_plan server\n");

	printf("Waiting for action server\n");
	client_action_.waitForServer();
	printf("Connected to action server\n");
}

void Supervisor::FSM()
{
	// modified only in : here
	switch(state_global_)
	{
		case GET_GOAL:
			printf("\n");
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
			printf("\n");
			ROS_INFO("ASK_PLAN");
			this->askPlan();
			plan_.show();
			state_global_ = EXEC_PLAN;
			break;

		case EXEC_PLAN:
			printf("\n");
			ROS_INFO("EXEC_PLAN");
			msg_.data = "SUPERVISOR STATE EXEC " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);
			printf("current_goal : %s (%f, %f, %f)\n", current_goal_.type.c_str(), current_goal_.x, current_goal_.y, current_goal_.theta);
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

					plan_.updateCurrentAction();
					std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

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
							current_path_.poses.clear();
							previous_path_.poses.clear();
							client_action_.sendGoal((*curr_action).action);
							this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
							(*curr_action).state=PROGRESS;
							break;

						case PROGRESS:
							printf("PROGRESS\n");
							// check postconditions
							// for now : if human at destination

							if(client_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
							{
								printf("Client succeeded\n");
								this->updateMarkerPose(0, 0, 0);
								current_path_.poses.clear();
								previous_path_.poses.clear();
								(*curr_action).state = DONE;
							}
							if(client_action_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
							{
								printf("PREEMPTED\n");
								this->updateMarkerPose(0, 0, 0);
								current_path_.poses.clear();
								previous_path_.poses.clear();
								state_global_=GET_GOAL;

							}
							else if(sqrt(pow(human_pose_.x-(*curr_action).action.target_pose.pose.position.x,2) + pow(human_pose_.y-(*curr_action).action.target_pose.pose.position.y,2)) > 0.5)
							{
								if(client_action_.getState() == actionlib::SimpleClientGoalState::LOST
								|| goal_aborted_count_==0 && (ros::Time::now() - last_replan_ > dur_replan_))
								{
									printf("=> Resend !\n");
									client_action_.sendGoal((*curr_action).action);
									this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
									last_replan_ = ros::Time::now();
								}
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
			msg_.data = "SUPERVISOR STATE BLOCKED " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			if(first_blocked_)
			{
				ROS_INFO("BLOCKED_BY_ROBOT");
				human_sim::CancelGoalAndStop srv;
				client_cancel_goal_and_stop_.call(srv);
				client_action_.stopTrackingGoal();

				last_replan_ = ros::Time::now();
				first_blocked_=false;
			}
			else
			{
				switch(blocked_state_)
				{
					case ABORTED:
					case LONGER:
						if(ros::Time::now() - last_replan_ > dur_replan_blocked_)
						{
							printf("try to replan\n");

							plan_.updateCurrentAction();
							std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

							nav_msgs::GetPlan srv;
							srv.request.start.pose.position.x = 	human_pose_.x;
							srv.request.start.pose.position.y = 	human_pose_.y;
							srv.request.start.header.frame_id = 	"map";
							srv.request.goal.pose.position.x = 	(*curr_action).action.target_pose.pose.position.x;
							srv.request.goal.pose.position.y = 	(*curr_action).action.target_pose.pose.position.y;
							srv.request.goal.header.frame_id = 	"map";
							srv.request.tolerance = 		0.1;

							// make plan
							if(client_make_plan_.call(srv))
							{
								if(srv.response.plan.poses.size()!=0) // successfully planned once
								{
									printf("BLOCK : srv.plan=%d previous=%d\n", (int)srv.response.plan.poses.size(), (int)previous_path_.poses.size());
									// if close enough to previous path
									if(abs((int)srv.response.plan.poses.size()-(int)previous_path_.poses.size()) < 10
									|| float(srv.response.plan.poses.size()) < 1.5*float(previous_path_.poses.size()))
									{
										if(replan_success_nb_ < 2)
										{
											replan_success_nb_++;
											printf("One success ! replan_success_nb = %d\n", replan_success_nb_);
										}
										else
										{
											printf("replan successfully !\n");
											replan_success_nb_ = 0;
											first_blocked_ = true;
											state_global_ = EXEC_PLAN;
										}
									}
									else
									{
										replan_success_nb_ = 0;
										printf("still blocked ..\n");
									}
								}
								else
								{
									printf("Failed to plan ...\n");
									replan_success_nb_ = 0;
								}
							}
							else
								ROS_ERROR("Failed to call service make_plan");

							last_replan_ = ros::Time::now();
						}
						break;

					case NOT_FEASIBLE:
					{
						// try to move, check if actually moving
						// if moving switch to exec plan
						// else keep trying to move as blocked

						plan_.updateCurrentAction();
						std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

						static Pose2D last_human_pose = human_pose_;

						if(first_not_feasible_)
						{
							last_human_pose = human_pose_;
							first_not_feasible_=false;
						}

						if(ros::Time::now() - last_replan_ > dur_replan_)
						{
							printf("send goal\n");
							client_action_.sendGoal((*curr_action).action);
							this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
							last_replan_ = ros::Time::now();

							if(abs(human_pose_.x - last_human_pose.x) > 0.03
							&& abs(human_pose_.y - last_human_pose.y) > 0.03)
							{
								printf("We moved !\n");
								first_blocked_ = true;
								first_not_feasible_ = true;
								state_global_ = EXEC_PLAN;
							}
						}
						break;
					}

					default:
						blocked_state_ = ABORTED;
						break;
				}
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

	static Pose2D last_human_pose = human_pose_;
	static ros::Time last_check_human_pose = ros::Time::now();
	static int same_human_pose_count = 0;

	printf("check : current=%d previous=%d\n", (int)current_path_.poses.size(), (int)previous_path_.poses.size());

	// Check goal aborted (failure computing a plan)
	if(state==actionlib::SimpleClientGoalState::ABORTED)
	{
		if(goal_aborted_count_ < 3)
			goal_aborted_count_++;
		else
		{
			printf("Checked ABORTED\n");
			goal_aborted_count_ = 0;
			current_path_.poses.clear();
			blocked_state_ = ABORTED;
			return true;
		}
	}
	else
		goal_aborted_count_ = 0;

	// Check if path changed too much
	if(previous_path_.poses.size() != 0 && current_path_.poses.size() != 0)
	{
		if(abs((int)current_path_.poses.size()-(int)previous_path_.poses.size()) > 20
		&& float(current_path_.poses.size())/float(previous_path_.poses.size()) > 1.5)
		{
			printf("Checked CHANGED TOO MUCH\n");
			current_path_.poses.clear();
			blocked_state_ = LONGER;
			return true;
		}
	}

	// Check if trajectory not feasible, thus if not moving for too long
	if(ros::Time::now() - last_check_human_pose > dur_check_pose_blocked_)
	{
		// if current pose is close enough to previous one
		if(abs(human_pose_.x - last_human_pose.x) < 0.03
		&& abs(human_pose_.y - last_human_pose.y) < 0.03
		&& abs(human_pose_.theta - last_human_pose.theta) < 0.02)
		{
			same_human_pose_count++;
			printf("SAME %d\n", same_human_pose_count);
		}
		else
			same_human_pose_count=0;

		last_human_pose.x = 	human_pose_.x;
		last_human_pose.y = 	human_pose_.y;
		last_human_pose.theta = human_pose_.theta;

		last_check_human_pose = ros::Time::now();

		if(same_human_pose_count > 3)
		{
			printf("Checked NOT FEASIBLE\n");
			same_human_pose_count = 0;
			blocked_state_ = NOT_FEASIBLE;
			return true;
		}
	}

	return false;
}

void Supervisor::updateMarkerPose(float x, float y, float alpha)
{
	marker_rviz_.pose.position.x = 	x;
	marker_rviz_.pose.position.y = 	y;
	marker_rviz_.color.a = 		alpha;

	pub_marker_rviz_.publish(marker_rviz_);
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

	goal_received_ = true;
	first_blocked_ = true;
	current_path_.poses.clear();
	previous_path_.poses.clear();

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
	printf("GET_GOAL_SET !!!\n");
	state_global_=GET_GOAL;
	current_path_.poses.clear();
	previous_path_.poses.clear();
	choice_goal_decision_ = SPECIFIED;

	return true;
}

bool Supervisor::getChoiceGoalDecision(human_sim::GetChoiceGoalDecision::Request &req, human_sim::GetChoiceGoalDecision::Response &res)
{
	res.decision = (int)choice_goal_decision_;

	return true;
}

void Supervisor::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
	printf("pathCallback ! %d \n", (int)path->poses.size());
	msg_.data = "SUPERVISOR " + std::to_string(path->header.stamp.toSec()) + " " + std::to_string(path->poses.size());
	pub_log_.publish(msg_);

	if(state_global_ != BLOCKED_BY_ROBOT)
	{
		printf("before CB : path=%d current=%d previous=%d\n", (int)path->poses.size(), (int)current_path_.poses.size(), (int)previous_path_.poses.size());
		if(current_path_.poses.size()==0 && previous_path_.poses.size()==0)
		{
			printf("======> first !\n");
			current_path_ = *path;
			msg_.data = "SUPERVISOR FIRST " + std::to_string(path->header.stamp.toSec()) + " " + std::to_string(path->poses.size());
			pub_log_.publish(msg_);
		}

		else if(current_path_.poses.size()==0 && previous_path_.poses.size()!=0)
		{
			printf("retreive new current path\n");
			current_path_ = *path;
		}

		else if(current_path_.poses.size()!=0)
		{
			printf("CUTTING path !\n");
			// seek pose closest to current_pose
			// only keep path from current_pose to the end
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

			current_path_ = *path;
		}

		printf("after CB : current=%d previous=%d\n", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
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

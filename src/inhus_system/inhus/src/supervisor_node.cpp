#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor()
: plan_()
, replan_freq_(1) // init Rates and durations
, check_freq_(1)
, place_robot_delay_(1)
{
	// Ros Params
	float f_nb;
	nh_.param(std::string("replan_active"), replan_active_, true);
	nh_.param(std::string("replan_freq"), f_nb, float(2.0)); replan_freq_ = ros::Rate(f_nb);
	nh_.param(std::string("replan_dist_stop"), replan_dist_stop_, float(0.5));
	nh_.param(std::string("check_freq"), f_nb, float(10)); check_freq_ = ros::Rate(f_nb);
	nh_.param(std::string("place_robot_delay"), f_nb, float(0.3)); place_robot_delay_ = ros::Duration(f_nb);
	ROS_INFO("SUP: => Params Supervisor :");
	ROS_INFO("SUP: replan_active=%d", replan_active_);
	ROS_INFO("SUP: replan_freq=%f", 1/replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("SUP: replan_dist_stop=%f", replan_dist_stop_);
	ROS_INFO("SUP: check_freq=%f", 1/check_freq_.expectedCycleTime().toSec());
	ROS_INFO("SUP: place_robot_delay=%f", place_robot_delay_.toSec());

	// Service clients
	client_plan_ = 						nh_.serviceClient<inhus::ComputePlan>("compute_plan");
	client_place_robot_hm_ =			nh_.serviceClient<inhus_navigation::PlaceRobot>("place_robot_hm");
	client_check_conflict_ =			nh_.serviceClient<inhus::ActionBool>("check_conflict");
	client_init_check_conflict_ = 		nh_.serviceClient<std_srvs::Empty>("init_check_conflict");
	client_init_first_path_conflict_ = 	nh_.serviceClient<inhus::ActionBool>("init_first_path_conflict");

	// Service servers
	server_set_wait_goal_ =		nh_.advertiseService("set_wait_goal", &Supervisor::srvSetWaitGoal, this);
	server_suspend_ =			nh_.advertiseService("suspendSupervisor", &Supervisor::srvSuspend, this);
	server_resume_ =	nh_.advertiseService("resumeSupervisor", &Supervisor::srvResume, this);

	// Subscribers
	sub_new_goal_  = 		nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_path_ =				nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);
	sub_status_move_base_ = nh_.subscribe("move_base/status", 100, &Supervisor::stateMoveBaseCB, this);

	// Publishers
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_goal_done_ = 	nh_.advertise<inhus::Goal>("goal_done", 100);
	pub_log_ =			nh_.advertise<std_msgs::String>("log", 100);
	pub_marker_rviz_ =	nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);


	// Init
	marker_rviz_.header.frame_id = 		"map";
	marker_rviz_.type = 			1;
	marker_rviz_.pose.position.x = 		0;
	marker_rviz_.pose.position.y = 		0;
	marker_rviz_.pose.position.z = 		0.25;
	marker_rviz_.scale.x = 			0.3;
	marker_rviz_.scale.y = 			0.3;
	marker_rviz_.scale.z = 			0.5;
	marker_rviz_.color.r = 			1;
	marker_rviz_.color.g = 			1;
	marker_rviz_.color.b = 			0;
	marker_rviz_.color.a = 			0;

	global_state_ = WAIT_GOAL;
	global_state_previous_ = SUSPENDED;
	goal_received_ = false;
	goal_status_.status = 0;
}

move_base_msgs::MoveBaseGoal Supervisor::getMoveBaseGoal(geometry_msgs::Pose2D pose)
{
	move_base_msgs::MoveBaseGoal move_base_goal;
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.pose.position.x = pose.x;
	move_base_goal.target_pose.pose.position.y = pose.y;
	tf2::Quaternion q;
	q.setRPY(0,0,pose.theta);
	move_base_goal.target_pose.pose.orientation.x =	q.x();
	move_base_goal.target_pose.pose.orientation.y =	q.y();
	move_base_goal.target_pose.pose.orientation.z =	q.z();
	move_base_goal.target_pose.pose.orientation.w =	q.w();
	return move_base_goal;
}

void Supervisor::FSM()
{
	this->statePrint();
	switch(global_state_)
	{
		case WAIT_GOAL:
			// Wait for goal from HumanBehaviorModel
			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
			}
			break;

		case ASK_PLAN:
		{
			std_srvs::Empty srv_init_conflict;
			client_init_check_conflict_.call(srv_init_conflict);
			// remove robot
			srv_place_robot_hm_.request.data = false;
			client_place_robot_hm_.call(srv_place_robot_hm_);
			place_robot_delay_.sleep();
			place_robot_delay_.sleep();
			this->askPlan();
			plan_.show();
			global_state_ = EXEC_PLAN;
			break;
		}

		case EXEC_PLAN:
			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
			}
			else
			{
				//plan_.show();
				if(plan_.isDone())
				{
					ROS_INFO("SUP: Plan is DONE !");
					pub_goal_done_.publish(current_goal_);
					plan_.clear();
					current_path_.poses.clear();
					global_state_ = WAIT_GOAL;
				}
				else
				{
					plan_.updateCurrentAction();
					std::vector<inhus::Action>::iterator curr_action = plan_.getCurrentAction();
					current_action_ = (*curr_action);

					if((*curr_action).type == "nav_action")
					{
						switch((*curr_action).state)
						{
							case STATE_PLANNED:
							case STATE_NEEDED:
								//ROS_INFO("SUP: PLANNED");

								// check preconditions
								// => for now no checking
								//if(precond==ok)
									(*curr_action).state=STATE_READY;
								//else
								//	(*curr_action).state=NEEDED;
								break;

							case STATE_READY:
							{
								//ROS_INFO("SUP: READY");
								std_srvs::Empty srv_init_conflict;
								client_init_check_conflict_.call(srv_init_conflict);

								(*curr_action) = computeActionWithRadius((*curr_action));
								
								inhus::ActionBool srv_init_first_path_conflict;
								move_base_msgs::MoveBaseGoal move_base_goal = getMoveBaseGoal((*curr_action).nav_action.pose);
								srv_init_first_path_conflict.request.action = move_base_goal;
								client_init_first_path_conflict_.call(srv_init_first_path_conflict);

								// plan without robot first //
								// send to geometric planner
								move_base_msgs::MoveBaseActionGoal nav_goal;
								nav_goal.goal.target_pose.header.frame_id = "map";
								nav_goal.goal.target_pose.header.stamp = ros::Time::now();
								nav_goal.goal = move_base_goal;
								pub_goal_move_base_.publish(nav_goal);

								this->updateMarkerPose((*curr_action).nav_action.pose.x, (*curr_action).nav_action.pose.y, 1);
								last_replan_ = ros::Time::now();

								// place robot back
								srv_place_robot_hm_.request.data = true;
								client_place_robot_hm_.call(srv_place_robot_hm_);
								place_robot_delay_.sleep(); // wait delay

								(*curr_action).state=STATE_PROGRESS;
								break;
							}

							case STATE_PROGRESS:
								//ROS_INFO("SUP: PROGRESS");
								msg_.data = "SUPERVISOR STATE PROGRESS " + std::to_string(ros::Time::now().toSec());
								pub_log_.publish(msg_);

								// check postconditions
								// for now : if geometric planner tells action is done
								if(goal_status_.status == 3) // SUCCEEDED
								{
									ROS_INFO("SUP: Client succeeded");
									this->updateMarkerPose(0, 0, 0);
									(*curr_action).state = STATE_DONE;
								}
								else
								{
									// If replaning is active check if we can replan
									if(replan_active_)
									{
										// If not too close, try to replan
										if((int)current_path_.poses.size()==0 || computePathLength(&current_path_) > replan_dist_stop_)
										{
											//ROS_INFO("SUP: Test for resend");
											if(goal_status_.status == actionlib::SimpleClientGoalState::LOST
											|| (ros::Time::now() - last_replan_ > replan_freq_.expectedCycleTime()))
											{
												// ROS_INFO("SUP: => Resend !");

												move_base_msgs::MoveBaseActionGoal nav_goal;
												nav_goal.goal.target_pose.header.frame_id = "map";
												nav_goal.goal.target_pose.header.stamp = ros::Time::now();
												nav_goal.goal = getMoveBaseGoal((*curr_action).nav_action.pose);
												pub_goal_move_base_.publish(nav_goal);

												this->updateMarkerPose((*curr_action).nav_action.pose.x, (*curr_action).nav_action.pose.y, 1);
												last_replan_ = ros::Time::now();
											}
										}
									}

									// Check if blocked
									if(ros::Time::now() - last_check_ > check_freq_.expectedCycleTime())
									{
										ros::spinOnce();
										inhus::ActionBool srv;
										srv.request.action = getMoveBaseGoal((*curr_action).nav_action.pose);
										client_check_conflict_.call(srv);
										if(srv.response.conflict)
										{
											ROS_INFO("SUP: CONFLICT !");
											global_state_ = SUSPENDED;
										}
										last_check_ = ros::Time::now();
									}
								}
								break;
						}
					}
					else if((*curr_action).type == "wait_action")
					{
						switch((*curr_action).state)
						{
							case STATE_PLANNED:
							case STATE_NEEDED:
							case STATE_READY:
								ROS_INFO("SUP: Start waiting action %f", (*curr_action).wait_action.duration);
								start_time_wait_action_ = ros::Time::now();
								dur_wait_action_ = ros::Duration((*curr_action).wait_action.duration);
								(*curr_action).state = STATE_PROGRESS;
								break;

							case STATE_PROGRESS:
								if(ros::Time::now() - start_time_wait_action_ > dur_wait_action_)
								{
									ROS_INFO("SUP: Waiting done");
									(*curr_action).state = STATE_DONE;
								}
								break;
						}
					}
					else
						ROS_ERROR("Unkown action type");

					plan_.updateState();
				}
			}
			break;

		case SUSPENDED:
			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
			}
			break;
	}
}

void Supervisor::statePrint()
{
	if(global_state_ != global_state_previous_)
	{
		switch(global_state_)
		{
			case WAIT_GOAL:
				ROS_INFO("SUP: \t => WAIT_GOAL <=");
				break;

			case ASK_PLAN:
				ROS_INFO("SUP: \t => ASK_PLAN <=");
				break;

			case EXEC_PLAN:
				ROS_INFO("SUP: \t => EXEC_PLAN <=");
				//ROS_INFO("SUP: current_goal : %s (%f, %f, %f)", current_goal_.type.c_str(), current_goal_.x, current_goal_.y, current_goal_.theta);
				break;

			case SUSPENDED:
				ROS_INFO("SUP: \t => SUSPENDED <=");
				break;
		}
	}
	global_state_previous_ = global_state_;
}

void Supervisor::updateMarkerPose(float x, float y, float alpha)
{
	marker_rviz_.pose.position.x = 	x;
	marker_rviz_.pose.position.y = 	y;
	marker_rviz_.color.a = 		alpha;

	pub_marker_rviz_.publish(marker_rviz_);
}

void Supervisor::askPlan()
{
	plan_.clear();

	inhus::ComputePlan srv;
	srv.request.goal = current_goal_;
	ros::service::waitForService("compute_plan");
	while(!client_plan_.call(srv))
	{
		ROS_ERROR("Failure while asking for a plan, asking again in 1s");
		ros::Duration(1).sleep();
	}
	for(int i=0; i<srv.response.actions.size(); i++)
		plan_.addAction(srv.response.actions[i]);
	plan_.updateState();
}

void Supervisor::newGoalCallback(const inhus::GoalConstPtr& msg)
{
	ROS_INFO("New goal received !");
	ROS_INFO("Goal:%s", msg->type.c_str());
	goal_received_ = true;
	current_goal_ = *msg;
}

bool Supervisor::srvSetWaitGoal(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO("SUP: WAIT_GOAL SET !!!");
	global_state_ = WAIT_GOAL;
	goal_received_ = false;

	return true;
}

bool Supervisor::srvSuspend(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	global_state_=SUSPENDED;
	return true;
}

bool Supervisor::srvResume(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	global_state_=EXEC_PLAN;

	move_base_msgs::MoveBaseActionGoal nav_goal;
	nav_goal.goal.target_pose.header.frame_id = "map";
	nav_goal.goal.target_pose.header.stamp = ros::Time::now();
	nav_goal.goal = getMoveBaseGoal(current_action_.nav_action.pose);
	pub_goal_move_base_.publish(nav_goal);

	last_replan_ = ros::Time::now();
	return true;
}

void Supervisor::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
	current_path_ = *path;
}

void Supervisor::stateMoveBaseCB(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
	if(!status->status_list.empty())
		goal_status_.status = status->status_list.back().status;
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

#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor()
: plan_()
, replan_freq_(1) // init Rates
, blocked_ask_path_freq_(1)
, not_feasible_check_pose_freq_(1)
, approach_freq_(1)
, place_robot_delay_(1)
{
	// Ros Params
	ros::NodeHandle private_nh("~");
	float f_nb;
	private_nh.param(std::string("replan_freq"), f_nb, float(2.0)); replan_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("replan_dist_stop"), replan_dist_stop_, float(0.5));
	private_nh.param(std::string("not_feasible_check_pose_freq"), f_nb, float(10.0)); not_feasible_check_pose_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("not_feasible_nb_same_pose_block"), not_feasible_nb_same_pose_block_, int(3));
	private_nh.param(std::string("not_feasible_dist_threshold_unblock"), not_feasible_dist_threshold_unblock_, float(0.05));
	private_nh.param(std::string("not_feasible_theta_threshold_unblock"), not_feasible_theta_threshold_unblock_, float(0.02));
	private_nh.param(std::string("blocked_ask_path_freq"), f_nb, float(2.0)); blocked_ask_path_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("blocked_nb_ask_success_unblock"), blocked_nb_ask_success_unblock_, int(1));
	private_nh.param(std::string("absolute_path_length_diff"), absolute_path_length_diff_, float(1.0));
	private_nh.param(std::string("ratio_path_length_diff"), ratio_path_length_diff_, float(1.5));
	private_nh.param(std::string("approach_dist"), approach_dist_, float(4.0));
	private_nh.param(std::string("approach_freq"), f_nb, float(2.0)); approach_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("place_robot_delay"), f_nb, float(0.3)); place_robot_delay_ = ros::Duration(f_nb);

	//ROS_INFO("Params:");
	//ROS_INFO("replan_freq=%f", replan_freq_.expectedCycleTime().toSec());
	//ROS_INFO("replan_dist_stop=%f", replan_dist_stop_);
	//ROS_INFO("not_feasible_check_pose_freq=%f", not_feasible_check_pose_freq_.expectedCycleTime().toSec());
	//ROS_INFO("not_feasible_nb_same_pose_block=%d", not_feasible_nb_same_pose_block_);
	//ROS_INFO("not_feasible_dist_threshold_unblock=%f", not_feasible_dist_threshold_unblock_);
	//ROS_INFO("not_feasible_theta_threshold_unblock=%f", not_feasible_theta_threshold_unblock_);
	//ROS_INFO("blocked_ask_path_freq=%f", blocked_ask_path_freq_.expectedCycleTime().toSec());
	//ROS_INFO("blocked_nb_ask_success_unblock=%d", blocked_nb_ask_success_unblock_);
	//ROS_INFO("absolute_path_length_diff=%f", absolute_path_length_diff_);
	//ROS_INFO("ratio_path_length_diff=%f", ratio_path_length_diff_);
	//ROS_INFO("approach_dist=%f", approach_dist_);
	//ROS_INFO("approach_freq=%f", approach_freq_.expectedCycleTime().toSec());
	//ROS_INFO("place_robot_delay=%f", place_robot_delay_.toSec());

	// Service clients
	client_plan_ = 					nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_make_plan_ =				nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
	client_cancel_goal_and_stop_= 	nh_.serviceClient<human_sim::Signal>("cancel_goal_and_stop");
	client_place_robot_hm_ =		nh_.serviceClient<move_human::PlaceRobot>("place_robot_hm");
	client_check_conflict_ =		nh_.serviceClient<human_sim::ActionBool>("check_conflict");
	client_init_check_conflict_ = 	nh_.serviceClient<human_sim::Signal>("init_check_conflict");

	// Services
	srv_get_plan_.request.start.header.frame_id = 	"map";
	srv_get_plan_.request.goal.header.frame_id = 	"map";
	srv_get_plan_.request.tolerance = 		0.1;

	// Subscribers
	sub_human_pose_ = 		nh_.subscribe("known/human_pose", 100, &Supervisor::humanPoseCallback, this);
	sub_robot_pose_ = 		nh_.subscribe("known/robot_pose", 100, &Supervisor::robotPoseCallback, this);
	sub_new_goal_  = 		nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_path_ =				nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);
	sub_status_move_base_ = nh_.subscribe("move_base/status", 100, &Supervisor::stateMoveBaseCB, this);

	// Publishers
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_cancel_goal_ =	nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);
	pub_log_ =			nh_.advertise<std_msgs::String>("log", 100);
	pub_marker_rviz_ =	nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	pub_stop_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("stop_cmd", 100);

	// Service servers
	service_set_get_goal_ =		nh_.advertiseService("set_get_goal", &Supervisor::setGetGoal, this);
	service_suspend_ =			nh_.advertiseService("suspend", &Supervisor::srvSuspend, this);

	// Init
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

	global_state_ = GET_GOAL;

	goal_received_ = false;

	goal_status_.status = 0;

	this->init();

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	ros::service::waitForService("compute_plan");
	//ROS_INFO("Connected to taskPlanner server");

	ros::service::waitForService("move_base/GlobalPlanner/make_plan");
	//ROS_INFO("Connected to make_plan server");
}

void Supervisor::init()
{
	replan_success_nb_ = 	0;
	last_replan_ = 		ros::Time::now();
	this->initCheckBlocked();
}

/*{
	current_path_.poses.clear();
	previous_path_.poses.clear();

	goal_aborted_count_ = 	0;

	same_human_pose_count_ = 0;
	last_human_pose_ = human_pose_;
	last_check_human_pose_ = ros::Time::now();
}*/

void Supervisor::FSM()
{
	switch(global_state_)
	{
		case GET_GOAL:
			ROS_INFO("\t => GET_GOAL <=");
			// Wait for goal from HumanBehaviorModel
			if(goal_received_)
			{
				goal_received_ = false;
				srv_place_robot_hm_.request.data = true;
				client_place_robot_hm_.call(srv_place_robot_hm_);
				global_state_ = ASK_PLAN;
			}
			else
				pub_stop_cmd_.publish(geometry_msgs::Twist());
			break;

		case ASK_PLAN:
			ROS_INFO("\t => ASK_PLAN <=");
			this->askPlan();
			plan_.show();
			global_state_ = EXEC_PLAN;
			break;

		case EXEC_PLAN:
			ROS_INFO("\t => EXEC_PLAN <=");
			//ROS_INFO("current_goal : %s (%f, %f, %f)", current_goal_.type.c_str(), current_goal_.x, current_goal_.y, current_goal_.theta);
			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
				srv_place_robot_hm_.request.data = true;
				client_place_robot_hm_.call(srv_place_robot_hm_);
			}
			else
			{
				plan_.show();
				if(plan_.isDone())
				{
					ROS_INFO("Plan is DONE !");
					pub_goal_done_.publish(current_goal_);
					plan_.clear();
					current_path_.poses.clear();
					global_state_ = GET_GOAL;
				}
				else
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
							ROS_INFO("NEEDED");
							// check preconditions
							// => for now no checking

							//if(precond==ok)
								(*curr_action).state=READY;
							//else
							//	(*curr_action).state=NEEDED;
							break;

						case READY:
						{
							ROS_INFO("READY");
							human_sim::Signal srv_init_conflict;
							client_init_check_conflict_.call(srv_init_conflict);

							// plan without robot first
							//ROS_INFO("Plan without robot");

							// remove robot
							srv_place_robot_hm_.request.data = false;
							client_place_robot_hm_.call(srv_place_robot_hm_);
							//ROS_INFO("removed");
							place_robot_delay_.sleep(); // wait delay

							// send to geometric planner
							move_base_msgs::MoveBaseActionGoal goal;
							goal.goal.target_pose.header.frame_id = "map";
							goal.goal.target_pose.header.stamp = ros::Time::now();
							goal.goal = (*curr_action).action;
							pub_goal_move_base_.publish(goal);

							this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x,
									(*curr_action).action.target_pose.pose.position.y, 1);
							last_replan_ = ros::Time::now();

							// place robot back
							srv_place_robot_hm_.request.data = true;
							client_place_robot_hm_.call(srv_place_robot_hm_);
							//ROS_INFO("robot back");
							place_robot_delay_.sleep(); // wait delay

							(*curr_action).state=PROGRESS;
							break;
						}

						case PROGRESS:
							ROS_INFO("PROGRESS");
							msg_.data = "SUPERVISOR STATE PROGRESS " + std::to_string(ros::Time::now().toSec());
							pub_log_.publish(msg_);

							// check postconditions
							// for now : if geometric planner tells action is done
							if(goal_status_.status == 3) // SUCCEEDED
							{
								ROS_INFO("Client succeeded");
								this->updateMarkerPose(0, 0, 0);
								(*curr_action).state = DONE;
							}
							else if(goal_status_.status == 2) // PREEMPTED
							{
								//ROS_INFO("PREEMPTED");
								this->updateMarkerPose(0, 0, 0);
								global_state_=GET_GOAL;

							}
							// If not too close, try to replan
							else if((int)current_path_.poses.size()==0 || computePathLength(&current_path_) > replan_dist_stop_)
							{
								//ROS_INFO("Test for resend");
								if(goal_status_.status == actionlib::SimpleClientGoalState::LOST
								|| (ros::Time::now() - last_replan_ > replan_freq_.expectedCycleTime()))
								{
									ROS_INFO("=> Resend !");

									move_base_msgs::MoveBaseActionGoal goal;
									goal.goal.target_pose.header.frame_id = "map";
									goal.goal.target_pose.header.stamp = ros::Time::now();
									goal.goal = (*curr_action).action;
									pub_goal_move_base_.publish(goal);

									this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x,
										(*curr_action).action.target_pose.pose.position.y, 1);
									//ros::Duration(0.2).sleep(); // investigate this sleep ? wait for path received to check
									last_replan_ = ros::Time::now();
								}
							}

							// Check if blocked
							ros::spinOnce();
							human_sim::ActionBool srv;
							srv.request.action = (*curr_action).action;
							client_check_conflict_.call(srv);
							if(srv.response.conflict)
							{
								ROS_INFO("CONFLICT !");
								global_state_ = SUSPENDED;
							}


							/*if(this->checkBlocked())
							{
								// stop human
								client_cancel_goal_and_stop_.call(srv_cancel_stop_);
								client_move_base_.stopTrackingGoal();

								// remove robot
								srv_place_robot_hm_.request.data = false;
								client_place_robot_hm_.call(srv_place_robot_hm_);
								place_robot_delay_.sleep();

								last_replan_ = ros::Time::now() - approach_freq_.expectedCycleTime();
								// wait before replanning to be sure the robot has actually been removed from the map

								// switch to APPROACH
								global_state_ = APPROACH;
								approach_state_ = REPLANNING;
							}*/
							break;
					}

					plan_.updateState();
				}
			}
			break;

		/*case APPROACH:
			msg_.data = "SUPERVISOR STATE APPROACH " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			if(goal_received_)
			{
				//ROS_INFO("APPROACH new goal");
				goal_received_ = false;
				global_state_ = ASK_PLAN;

				// be sure robot is on the map
				srv_place_robot_hm_.request.data = true;
				client_place_robot_hm_.call(srv_place_robot_hm_);
			}
			else
			{
				// if close enough => BLOCKED
				float dist_to_robot = sqrt(pow(human_pose_.x - robot_pose_.x,2) + pow(human_pose_.y - robot_pose_.y,2));
				if(dist_to_robot <= approach_dist_)
				{
					//ROS_INFO("close enough");

					// switch to BLOCKED
					global_state_ = BLOCKED;

					// place back robot
					srv_place_robot_hm_.request.data = true;
					client_place_robot_hm_.call(srv_place_robot_hm_);
					place_robot_delay_.sleep();

					// stop human
					client_cancel_goal_and_stop_.call(srv_cancel_stop_);
					client_move_base_.stopTrackingGoal();

					last_replan_ = ros::Time::now() - blocked_ask_path_freq_.expectedCycleTime();
				}
				else
				{
					// get current action
					plan_.updateCurrentAction();
					std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

					switch(approach_state_)
					{
						// Replan without the robot
						case REPLANNING:
							if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
							{
								//ROS_INFO("\t => APPROACH <=");
								//ROS_INFO("REPLANNING");
								//ROS_INFO("dist=%f", dist_to_robot);

								// if not too close from approach_dist
								if(dist_to_robot > approach_dist_ + replan_dist_stop_)
								{
									client_move_base_.sendGoal((*curr_action).action);
									this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
								}

								// put the robot back on the map to check if still blocked in next approach loop
								srv_place_robot_hm_.request.data = true;
								client_place_robot_hm_.call(srv_place_robot_hm_);
								approach_state_ = CHECKING;

								last_replan_ = ros::Time::now();
							}
							break;

						// check, with the robot, if the human is still blocked
						case CHECKING:
							if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
							{
								//ROS_INFO("\t => APPROACH <=");
								//ROS_INFO("CHECKING");
								//ROS_INFO("dist=%f", dist_to_robot);

								// check if still blocked
								bool still_blocked = true;
								srv_get_plan_.request.start.pose.position.x = 	human_pose_.x;
								srv_get_plan_.request.start.pose.position.y = 	human_pose_.y;
								srv_get_plan_.request.goal.pose.position.x = 	(*curr_action).action.target_pose.pose.position.x;
								srv_get_plan_.request.goal.pose.position.y = 	(*curr_action).action.target_pose.pose.position.y;
								if(client_make_plan_.call(srv_get_plan_))
								{
									//ROS_INFO("make_plan %d", (int)srv_get_plan_.response.plan.poses.size());
									if((int)srv_get_plan_.response.plan.poses.size()!=0) // path found
									{
										float response_path_length = this->computePathLength(&(srv_get_plan_.response.plan));
										float previous_path_length = this->computePathLength(&previous_path_);

										// check if the path found is 'good'
										if(previous_path_length == 0								// if no path was found before
										|| abs(response_path_length-previous_path_length)<absolute_path_length_diff_		// if close enough in absolute
										|| response_path_length < ratio_path_length_diff_*previous_path_length)   		// or if clone enough relatively
											still_blocked = false;
									}
								}
								else
									ROS_ERROR("Failed to call service make_plan");

								if(still_blocked)
								{
									//ROS_INFO("Still blocked, rm R");

									// remove the robot from the map for replanning in next approach loop
									srv_place_robot_hm_.request.data = false;
									client_place_robot_hm_.call(srv_place_robot_hm_);
									last_replan_ = ros::Time::now();
									approach_state_ = REPLANNING;
								}
								else
								{
									// since the human isn't blocked anymore, switch back to EXEC_PLAN
									last_replan_ = ros::Time::now() - replan_freq_.expectedCycleTime();
									global_state_ = EXEC_PLAN;
								}
							}
							break;
					}
				}
			}
			break;

		case BLOCKED:
			msg_.data = "SUPERVISOR STATE BLOCKED " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
				srv_place_robot_hm_.request.data = true;
				client_place_robot_hm_.call(srv_place_robot_hm_);
			}
			else
			{
				// if too far, back to APPROACH
				float dist_to_robot = sqrt(pow(human_pose_.x - robot_pose_.x,2) + pow(human_pose_.y - robot_pose_.y,2));
				//ROS_INFO("dist=%f", dist_to_robot);
				if(dist_to_robot > approach_dist_)
				{
					// remove robot
					srv_place_robot_hm_.request.data = false;
					client_place_robot_hm_.call(srv_place_robot_hm_);

					last_replan_ = ros::Time::now() - approach_freq_.expectedCycleTime();
					// wait before replanning to be sure the robot has actually been removed from the map

					// switch to APPROACH
					global_state_ = APPROACH;
					approach_state_ = REPLANNING;
				}
				else
				{
					switch(blocked_state_)
					{
						case ABORTED:
						case LONGER:
							if(ros::Time::now() - last_replan_ > blocked_ask_path_freq_.expectedCycleTime())
							{
								//ROS_INFO("\t => BLOCKED <=");
								//ROS_INFO("try to replan");

								plan_.updateCurrentAction();
								std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

								srv_get_plan_.request.start.pose.position.x = 	human_pose_.x;
								srv_get_plan_.request.start.pose.position.y = 	human_pose_.y;
								srv_get_plan_.request.goal.pose.position.x = 	(*curr_action).action.target_pose.pose.position.x;
								srv_get_plan_.request.goal.pose.position.y = 	(*curr_action).action.target_pose.pose.position.y;

								// make plan
								if(client_make_plan_.call(srv_get_plan_))
								{
									//ROS_INFO("BLOCK : srv.plan=%d previous=%d", (int)srv_get_plan_.response.plan.poses.size(), (int)previous_path_.poses.size());
									last_replan_ = ros::Time::now();
									if((int)srv_get_plan_.response.plan.poses.size()!=0) // successfully planned once
									{

										float response_path_length = this->computePathLength(&(srv_get_plan_.response.plan));
										float previous_path_length = this->computePathLength(&previous_path_);

										// If new path is 'good'
										if(previous_path_length == 0								// if no path was found before
										|| abs(response_path_length-previous_path_length)<absolute_path_length_diff_		// if close enough in absolute
										|| response_path_length < ratio_path_length_diff_*previous_path_length)   		// or if clone enough relatively
										{
											replan_success_nb_++;
											//ROS_INFO("One success ! replan_success_nb = %d", replan_success_nb_);

											if(replan_success_nb_ >= blocked_nb_ask_success_unblock_)
											{
												//ROS_INFO("replan successfully !");
												replan_success_nb_ = 0;
												last_replan_ = ros::Time::now() - replan_freq_.expectedCycleTime();
												global_state_ = EXEC_PLAN;
											}
										}
										else
										{
											replan_success_nb_ = 0;
											//ROS_INFO("still blocked ..");
										}
									}
									else
									{
										//ROS_INFO("Failed to plan ...");
										replan_success_nb_ = 0;
									}
								}
								else
									ROS_ERROR("Failed to call service make_plan");
							}
							break;

						case NOT_FEASIBLE:
						{
							// try to move, check if actually moving
							// if moving switch to exec plan
							// else keep trying to move as blocked

							plan_.updateCurrentAction();
							std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

							static geometry_msgs::Pose2D last_human_pose = human_pose_;

							if(first_not_feasible_)
							{
								last_human_pose = human_pose_;
								first_not_feasible_=false;
							}

							if(ros::Time::now() - last_replan_ > replan_freq_.expectedCycleTime())
							{
								//ROS_INFO("\t => BLOCKED <=");
								//ROS_INFO("not feasible, send goal");
								client_move_base_.sendGoal((*curr_action).action);
								this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
								last_replan_ = ros::Time::now();

								if(abs(human_pose_.x-last_human_pose.x) > not_feasible_dist_threshold_unblock_
								|| abs(human_pose_.y-last_human_pose.y) > not_feasible_dist_threshold_unblock_
								|| abs(human_pose_.theta-last_human_pose.theta) > not_feasible_theta_threshold_unblock_)
								{
									//ROS_INFO("We moved !");
									first_not_feasible_ = true;
									global_state_ = EXEC_PLAN;
								}
							}
							break;
						}

						default:
							blocked_state_ = ABORTED;
							break;
					}
				}
			}
			break;*/

		case SUSPENDED:
			ROS_INFO("\t => SUSPENDED <=");
			break;

		default:
			global_state_=GET_GOAL;
			break;
	}
}

bool Supervisor::checkBlocked()
{
	//ROS_INFO("checkBlocked");
	if(goal_status_.status == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("CLIENT STATE : SUCCEEDED");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::PENDING){
		ROS_INFO("CLIENT STATE : PENDING");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::ACTIVE){
		ROS_INFO("CLIENT STATE : ACTIVE");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::RECALLED){
		ROS_INFO("CLIENT STATE : RECALLED");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::REJECTED){
		ROS_INFO("CLIENT STATE : REJECTED");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::PREEMPTED){
		ROS_INFO("CLIENT STATE : PREEMPTED");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::ABORTED){
		ROS_INFO("CLIENT STATE : ABORTED");
	}
	else if(goal_status_.status == actionlib::SimpleClientGoalState::LOST){
		ROS_INFO("CLIENT STATE : LOST");
	}
	ROS_INFO("statusCheck=%d", goal_status_.status);
	ROS_INFO("PENDING = %d", actionlib::SimpleClientGoalState::PENDING);
	ROS_INFO("ACTIVE = %d", actionlib::SimpleClientGoalState::ACTIVE);
	ROS_INFO("SUCCEEDED = %d", actionlib::SimpleClientGoalState::SUCCEEDED);
	ROS_INFO("ABORTED = %d", actionlib::SimpleClientGoalState::ABORTED);
	ROS_INFO("LOST = %d", actionlib::SimpleClientGoalState::LOST);
	ROS_INFO("REJECTED = %d", actionlib::SimpleClientGoalState::REJECTED);

/*
	// Check goal aborted (no path found)
	if(state==actionlib::SimpleClientGoalState::ABORTED)
	{
		goal_aborted_count_++;
		//ROS_INFO("Aborted detected %d", goal_aborted_count_);
		if(goal_aborted_count_ >= 1)
		{
			//ROS_INFO("Checked ABORTED");
			goal_aborted_count_ = 0;
			current_path_.poses.clear();
			blocked_state_ = ABORTED;
			return true;
		}
	}
	else
		goal_aborted_count_ = 0;

	// Check if path changed too much
	//ROS_INFO("check : current=%d previous=%d", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
	if((int)previous_path_.poses.size() != 0 && (int)current_path_.poses.size() != 0)
	{
		float current_path_length = this->computePathLength(&current_path_);
		float previous_path_length = this->computePathLength(&previous_path_);

		if(abs(current_path_length-previous_path_length) > absolute_path_length_diff_ 	// if difference big enough in absolute
		&& current_path_length > ratio_path_length_diff_*previous_path_length)   			// and if difference big enough relatively
		{
			//ROS_INFO("Checked CHANGED TOO MUCH");
			current_path_.poses.clear();
			blocked_state_ = LONGER;
			return true;
		}
	}

	// Check if trajectory not feasible, thus if not moving for too long
	if(ros::Time::now() - last_check_human_pose_ > not_feasible_check_pose_freq_.expectedCycleTime() && false)
	{
		//ROS_INFO("check not feasible");
		// if current pose is close enough to previous one
		if(abs(human_pose_.x - last_human_pose_.x) < not_feasible_dist_threshold_unblock_
		&& abs(human_pose_.y - last_human_pose_.y) < not_feasible_dist_threshold_unblock_
		&& abs(human_pose_.theta - last_human_pose_.theta) < not_feasible_theta_threshold_unblock_)
		{
			same_human_pose_count_++;
			//ROS_INFO("SAME %d", same_human_pose_count_);
		}
		else
			same_human_pose_count_=0;

		last_human_pose_.x = 	human_pose_.x;
		last_human_pose_.y = 	human_pose_.y;
		last_human_pose_.theta = human_pose_.theta;

		last_check_human_pose_ = ros::Time::now();

		if(same_human_pose_count_ >= not_feasible_nb_same_pose_block_)
		{
			//ROS_INFO("Checked NOT FEASIBLE");
			same_human_pose_count_ = 0;
			blocked_state_ = NOT_FEASIBLE;
			return true;
		}
	}
*/
	return false;
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
	//ROS_INFO("New goal received!");

	goal_received_ = 	true;

	this->init();

	current_goal_.type=msg->type;
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

bool Supervisor::setGetGoal(human_sim::Signal::Request &req, human_sim::Signal::Response &res)
{
	//ROS_INFO("GET_GOAL_SET !!!");
	global_state_=GET_GOAL;

	this->init();

	return true;
}

bool Supervisor::srvSuspend(human_sim::Signal::Request &req, human_sim::Signal::Response &res)
{
	//ROS_INFO("GET_GOAL_SET !!!");
	global_state_=SUSPENDED;

	return true;
}

float Supervisor::computePathLength(const nav_msgs::Path* path)
{
	float length=0;

	int path_size = (int)path->poses.size();
	for(int i=0; i<path_size-1; i++)
		length += sqrt( pow(path->poses[i+1].pose.position.x-path->poses[i].pose.position.x,2) + pow(path->poses[i+1].pose.position.y-path->poses[i].pose.position.y,2) );

	return length;
}

void Supervisor::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
	current_path_ = *path;
}

int Supervisor::cutPath(const nav_msgs::Path& path1, nav_msgs::Path& path2)
{
	int i_min = -1;

	if((int)path1.poses.size()>0)
	{
		float dist = sqrt(pow(path1.poses[0].pose.position.x-human_pose_.x,2) + pow(path1.poses[0].pose.position.y-human_pose_.y,2));
		float dist_min = dist;
		i_min = 0;
		for(int i=1; i<(int)path1.poses.size(); i++)
		{
			dist = sqrt(pow(path1.poses[i].pose.position.x-human_pose_.x,2) + pow(path1.poses[i].pose.position.y-human_pose_.y,2));
			if(dist < dist_min)
			{
				dist_min = dist;
				i_min = i;
			}
		}

		path2.poses.clear();
		for(int i=i_min; i<(int)path1.poses.size(); i++)
			path2.poses.push_back(path1.poses[i]);
	}

	return i_min;
}

void Supervisor::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	human_pose_ = *msg;
}

void Supervisor::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	robot_pose_ = *msg;
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

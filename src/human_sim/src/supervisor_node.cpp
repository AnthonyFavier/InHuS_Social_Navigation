#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor()
: plan_()
, client_action_("move_base", true)
, replan_freq_(1) // init Rates 
, blocked_ask_path_freq_(1)
, not_feasible_check_pose_freq_(1)
, approach_freq_(1)
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

	ROS_INFO("Params:");
	ROS_INFO("replan_freq=%f", replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("replan_dist_stop=%f", replan_dist_stop_);
	ROS_INFO("not_feasible_check_pose_freq=%f", not_feasible_check_pose_freq_.expectedCycleTime().toSec());
	ROS_INFO("not_feasible_nb_same_pose_block=%d", not_feasible_nb_same_pose_block_);
	ROS_INFO("not_feasible_dist_threshold_unblock=%f", not_feasible_dist_threshold_unblock_);
	ROS_INFO("not_feasible_theta_threshold_unblock=%f", not_feasible_theta_threshold_unblock_);
	ROS_INFO("blocked_ask_path_freq=%f", blocked_ask_path_freq_.expectedCycleTime().toSec());
	ROS_INFO("blocked_nb_ask_success_unblock=%d", blocked_nb_ask_success_unblock_);
	ROS_INFO("absolute_path_length_diff=%f", absolute_path_length_diff_);
	ROS_INFO("ratio_path_length_diff=%f", ratio_path_length_diff_);
	ROS_INFO("approach_dist=%f", approach_dist_);
	ROS_INFO("approach_freq=%f", approach_freq_.expectedCycleTime().toSec());

	// Service clients
	client_plan_ = 			nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_make_plan_ =		nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
	client_cancel_goal_and_stop_= 	nh_.serviceClient<human_sim::CancelGoalAndStop>("cancel_goal_and_stop");
	client_place_robot_ =		nh_.serviceClient<move_human::PlaceRobot>("place_robot");

	// Subscribers
	sub_human_pose_ = 	nh_.subscribe("known/human_pose", 100, &Supervisor::humanPoseCallback, this);
	sub_robot_pose_ = 	nh_.subscribe("known/robot_pose", 100, &Supervisor::robotPoseCallback, this);
	sub_new_goal_  = 	nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_path_ =		nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);

	// Publishers
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);
	pub_log_ =		nh_.advertise<std_msgs::String>("log", 100);
	pub_marker_rviz_ =	nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	pub_stop_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("stop_cmd", 100);

	// Service servers
	service_set_get_goal_ =			nh_.advertiseService("set_get_goal", &Supervisor::setGetGoal, this);

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
	approach_state_ = FIRST;

	goal_received_ = false;

	this->init();

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	ros::service::waitForService("compute_plan");
	ROS_INFO("Connected to taskPlanner server");

	ros::service::waitForService("move_base/GlobalPlanner/make_plan");
	ROS_INFO("Connected to make_plan server");

	ROS_INFO("Waiting for action server");
	client_action_.waitForServer();
	ROS_INFO("Connected to action server");
}

void Supervisor::init()
{
	current_path_.poses.clear();
	previous_path_.poses.clear();
	first_not_feasible_ = 	true;
	first_blocked_ = 	true;
	replan_success_nb_ = 	0;
	goal_aborted_count_ = 	0;
	last_replan_ = 		ros::Time::now();
	this->initCheckBlocked();
}

void Supervisor::initCheckBlocked()
{
	same_human_pose_count_ = 0;
	last_human_pose_ = human_pose_;
	last_check_human_pose_ = ros::Time::now();
}

void Supervisor::FSM()
{
	// modified only in : here
	switch(global_state_)
	{
		case GET_GOAL:
			ROS_INFO("\t => GET_GOAL <=");
			// Wait for goal from HumanModel
			if(goal_received_)
			{
				goal_received_ = false;
				move_human::PlaceRobot srv;
				srv.request.data = true;
				client_place_robot_.call(srv);
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
			msg_.data = "SUPERVISOR STATE EXEC " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);
			ROS_INFO("current_goal : %s (%f, %f, %f)", current_goal_.type.c_str(), current_goal_.x, current_goal_.y, current_goal_.theta);
			if(goal_received_)
			{
				goal_received_ = false;
				global_state_ = ASK_PLAN;
				move_human::PlaceRobot srv;
				srv.request.data = true;
				client_place_robot_.call(srv);
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
					previous_path_.poses.clear();
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
							ROS_INFO("READY");
							// send to geometric planner
							current_path_.poses.clear();
							previous_path_.poses.clear();
							client_action_.sendGoal((*curr_action).action);
							this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
							this->initCheckBlocked();
							(*curr_action).state=PROGRESS;
							break;

						case PROGRESS:
							ROS_INFO("PROGRESS");
							// check postconditions
							// for now : if human at destination
							if(client_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
							{
								ROS_INFO("Client succeeded");
								this->updateMarkerPose(0, 0, 0);
								current_path_.poses.clear();
								previous_path_.poses.clear();
								(*curr_action).state = DONE;
							}
							else if(client_action_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
							{
								ROS_INFO("PREEMPTED");
								this->updateMarkerPose(0, 0, 0);
								current_path_.poses.clear();
								previous_path_.poses.clear();
								global_state_=GET_GOAL;

							}
							// If not too close, try to replan
							else if(computePathLength(&current_path_) > replan_dist_stop_)
							{
								ROS_INFO("Test for resend");
								if(client_action_.getState() == actionlib::SimpleClientGoalState::LOST
								|| goal_aborted_count_==0 && (ros::Time::now() - last_replan_ > replan_freq_.expectedCycleTime()))
								{
									ROS_INFO("=> Resend !");
									client_action_.sendGoal((*curr_action).action);
									this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
									last_replan_ = ros::Time::now();
								}
							}

							if(this->checkBlocked())
							{
								global_state_ = APPROACH;
								approach_state_ = FIRST;
							}
							break;
					}

					plan_.updateState();
				}
			}
			break;

		case APPROACH:
			msg_.data = "SUPERVISOR STATE APPROACH " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			if(goal_received_)
			{
				goal_received_ = false;
				move_human::PlaceRobot srv;
				srv.request.data = true;
				client_place_robot_.call(srv);
				global_state_ = ASK_PLAN;
			}
			else
			{
				// if close enough
				float dist_to_robot = sqrt(pow(human_pose_.x - robot_pose_.x,2) + pow(human_pose_.y - robot_pose_.y,2));
				if(dist_to_robot <= approach_dist_)
				{
					ROS_INFO("close enough");

					// switch to BLOCKED
					approach_state_ = CHECKING;
					global_state_ = BLOCKED;
					move_human::PlaceRobot srv;
					srv.request.data = true;
					client_place_robot_.call(srv);
				}
				else
				{	
					// get current action
					plan_.updateCurrentAction();
					std::vector<Action>::iterator curr_action = plan_.getCurrentAction();

					switch(approach_state_)
					{
						case FIRST:
						{
							ROS_INFO("\t => APPROACH <=");
							ROS_INFO("FIRST");
							ROS_INFO("dist=%f", dist_to_robot);

							move_human::PlaceRobot srv;
							srv.request.data = false;
							client_place_robot_.call(srv);

							ROS_INFO("waiting ...");
							ros::Duration(0.5).sleep(); // peut etre forcement stop goal et human dans check puis relance ici, pour eviter commence a suivre chemin long ...

							// replan 
							client_action_.sendGoal((*curr_action).action);
							this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
							last_replan_ = ros::Time::now();
							ROS_INFO("replanned");

							srv.request.data = true;
							client_place_robot_.call(srv);
	
							approach_state_ = CHECKING;
							break;
						}

						case CHECKING:
							if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
							{
								ROS_INFO("\t => APPROACH <=");
								ROS_INFO("CHECKING");
								ROS_INFO("dist=%f", dist_to_robot);

								// check if still blocked 
								bool still_blocked = true;
								nav_msgs::GetPlan srv;
								srv.request.start.pose.position.x = 	human_pose_.x;
								srv.request.start.pose.position.y = 	human_pose_.y;
								srv.request.start.header.frame_id = 	"map";
								srv.request.goal.pose.position.x = 	(*curr_action).action.target_pose.pose.position.x;
								srv.request.goal.pose.position.y = 	(*curr_action).action.target_pose.pose.position.y;
								srv.request.goal.header.frame_id = 	"map";
								srv.request.tolerance = 		0.1;
								if(client_make_plan_.call(srv))
								{
									ROS_INFO("make_plan %d", (int)srv.response.plan.poses.size());
									if((int)srv.response.plan.poses.size()!=0) // path found
									{
										float response_path_length = this->computePathLength(&(srv.response.plan));
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
									ROS_INFO("Still blocked, rm R");

									// remove the robot from the map for next replanning
									move_human::PlaceRobot srv;
									srv.request.data = false;
									client_place_robot_.call(srv);
									approach_state_ = REPLANNING;
								}
								else
								{
									// since the human isn't blocked anymore, switch back to EXEC_PLAN 
									last_replan_ = ros::Time::now() - replan_freq_.expectedCycleTime();
									approach_state_ = CHECKING;
									global_state_ = EXEC_PLAN;
								}

								last_replan_ = ros::Time::now();
							}
							break;

						case REPLANNING:
							if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
							{
								ROS_INFO("\t => APPROACH <=");
								ROS_INFO("REPLANNING");
								ROS_INFO("dist=%f", dist_to_robot);

								// replan (without the robot)
								if(dist_to_robot > approach_dist_ + 1) // if not too close from approach_dist
								{
									client_action_.sendGoal((*curr_action).action);
									this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
								}

								// put the robot back on the map to check if still blocked in next checking
								move_human::PlaceRobot srv;
								srv.request.data = true;
								client_place_robot_.call(srv);
								approach_state_ = CHECKING;

								last_replan_ = ros::Time::now();
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
				move_human::PlaceRobot srv;
				srv.request.data = true;
				client_place_robot_.call(srv);
			}
			else
			{
				if(first_blocked_)
				{
					ROS_INFO("\t => BLOCKED <=");
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
							if(ros::Time::now() - last_replan_ > blocked_ask_path_freq_.expectedCycleTime())
							{
								ROS_INFO("try to replan");

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
									ROS_INFO("BLOCK : srv.plan=%d previous=%d", (int)srv.response.plan.poses.size(), (int)previous_path_.poses.size());
									if((int)srv.response.plan.poses.size()!=0) // successfully planned once
									{

										float response_path_length = this->computePathLength(&(srv.response.plan));
										float previous_path_length = this->computePathLength(&previous_path_);

										// If new path is 'good'
										if(previous_path_length == 0								// if no path was found before
										|| abs(response_path_length-previous_path_length)<absolute_path_length_diff_		// if close enough in absolute
										|| response_path_length < ratio_path_length_diff_*previous_path_length)   		// or if clone enough relatively
										{
											replan_success_nb_++;
											ROS_INFO("One success ! replan_success_nb = %d", replan_success_nb_);

											if(replan_success_nb_ >= blocked_nb_ask_success_unblock_)
											{
												ROS_INFO("replan successfully !");
												replan_success_nb_ = 0;
												first_blocked_ = true;
												last_replan_ = ros::Time::now() - replan_freq_.expectedCycleTime();
												global_state_ = EXEC_PLAN;
											}
										}
										else
										{
											replan_success_nb_ = 0;
											ROS_INFO("still blocked ..");
										}
									}
									else
									{
										ROS_INFO("Failed to plan ...");
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

							static geometry_msgs::Pose2D last_human_pose = human_pose_;

							if(first_not_feasible_)
							{
								last_human_pose = human_pose_;
								first_not_feasible_=false;
							}

							if(ros::Time::now() - last_replan_ > replan_freq_.expectedCycleTime())
							{
								ROS_INFO("send goal");
								client_action_.sendGoal((*curr_action).action);
								this->updateMarkerPose((*curr_action).action.target_pose.pose.position.x, (*curr_action).action.target_pose.pose.position.y, 1);
								last_replan_ = ros::Time::now();

								if(abs(human_pose_.x-last_human_pose.x) > not_feasible_dist_threshold_unblock_
								|| abs(human_pose_.y-last_human_pose.y) > not_feasible_dist_threshold_unblock_
								|| abs(human_pose_.theta-last_human_pose.theta) > not_feasible_theta_threshold_unblock_)
								{
									ROS_INFO("We moved !");
									first_blocked_ = true;
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
			break;

		default:
			global_state_=GET_GOAL;
			break;
	}
}

bool Supervisor::checkBlocked()
{
	ROS_INFO("checkBlocked");
	actionlib::SimpleClientGoalState state = client_action_.getState();
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("CLIENT STATE : SUCCEEDED");
	else if(state == actionlib::SimpleClientGoalState::PENDING)
		ROS_INFO("CLIENT STATE : PENDING");
	else if(state == actionlib::SimpleClientGoalState::ACTIVE)
		ROS_INFO("CLIENT STATE : ACTIVE");
	else if(state == actionlib::SimpleClientGoalState::RECALLED)
		ROS_INFO("CLIENT STATE : RECALLED");
	else if(state == actionlib::SimpleClientGoalState::REJECTED)
		ROS_INFO("CLIENT STATE : REJECTED");
	else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
		ROS_INFO("CLIENT STATE : PREEMPTED");
	else if(state == actionlib::SimpleClientGoalState::ABORTED)
		ROS_INFO("CLIENT STATE : ABORTED");
	else if(state == actionlib::SimpleClientGoalState::LOST)
		ROS_INFO("CLIENT STATE : LOST");

	// Check goal aborted (no path found)
	if(state==actionlib::SimpleClientGoalState::ABORTED)
	{
		goal_aborted_count_++;
		ROS_INFO("Aborted detected %d", goal_aborted_count_);
		if(goal_aborted_count_ >= 1)
		{
			ROS_INFO("Checked ABORTED");
			goal_aborted_count_ = 0;
			current_path_.poses.clear();
			blocked_state_ = ABORTED;
			return true;
		}
	}
	else
		goal_aborted_count_ = 0;

	// Check if path changed too much
	ROS_INFO("check : current=%d previous=%d", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
	if((int)previous_path_.poses.size() != 0 && (int)current_path_.poses.size() != 0)
	{
		float current_path_length = this->computePathLength(&current_path_);
		float previous_path_length = this->computePathLength(&previous_path_);

		if(abs(current_path_length-previous_path_length) > absolute_path_length_diff_ 	// if difference big enough in absolute
		&& current_path_length > ratio_path_length_diff_*previous_path_length)   			// and if difference big enough relatively
		{
			ROS_INFO("Checked CHANGED TOO MUCH");
			current_path_.poses.clear();
			blocked_state_ = LONGER;
			return true;
		}
	}

	// Check if trajectory not feasible, thus if not moving for too long
	if(ros::Time::now() - last_check_human_pose_ > not_feasible_check_pose_freq_.expectedCycleTime())
	{
		ROS_INFO("check not feasible");
		// if current pose is close enough to previous one
		if(abs(human_pose_.x - last_human_pose_.x) < not_feasible_dist_threshold_unblock_
		&& abs(human_pose_.y - last_human_pose_.y) < not_feasible_dist_threshold_unblock_
		&& abs(human_pose_.theta - last_human_pose_.theta) < not_feasible_theta_threshold_unblock_)
		{
			same_human_pose_count_++;
			ROS_INFO("SAME %d", same_human_pose_count_);
		}
		else
			same_human_pose_count_=0;

		last_human_pose_.x = 	human_pose_.x;
		last_human_pose_.y = 	human_pose_.y;
		last_human_pose_.theta = human_pose_.theta;

		last_check_human_pose_ = ros::Time::now();

		if(same_human_pose_count_ >= not_feasible_nb_same_pose_block_)
		{
			ROS_INFO("Checked NOT FEASIBLE");
			same_human_pose_count_ = 0;
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

	goal_received_ = 	true;

	this->init();

	current_goal_.type=msg->type;
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

bool Supervisor::setGetGoal(human_sim::SetGetGoal::Request &req, human_sim::SetGetGoal::Response &res)
{
	ROS_INFO("GET_GOAL_SET !!!");
	global_state_=GET_GOAL;

	this->init();

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
	float path_length = this->computePathLength(path.get());

	msg_.data = "SUPERVISOR " + std::to_string(path->header.stamp.toSec()) + " " + std::to_string(path_length);
	pub_log_.publish(msg_);

	if(global_state_ != BLOCKED && global_state_ != APPROACH)
	{
		ROS_INFO("pathCallback %d ! length %f ", (int)path->poses.size(), path_length);

		ROS_INFO("before CB : path=%d current=%d previous=%d", (int)path->poses.size(), (int)current_path_.poses.size(), (int)previous_path_.poses.size());
		if((int)current_path_.poses.size()==0 && (int)previous_path_.poses.size()==0)
		{
			ROS_INFO("======> first !");
			current_path_ = *path;
			msg_.data = "SUPERVISOR FIRST " + std::to_string(path->header.stamp.toSec()) + " " + std::to_string(path_length);
			pub_log_.publish(msg_);
		}

		else if((int)current_path_.poses.size()==0 && (int)previous_path_.poses.size()!=0)
		{
			ROS_INFO("retreive new current path");
			current_path_ = *path;
		}

		else if((int)current_path_.poses.size()!=0)
		{
			ROS_INFO("CUTTING path !");

			// seek pose closest to current_pose from current_path
			// only keep path from current_pose to the end store to previous
			this->cutPath(current_path_, previous_path_);

			// update current_path
			current_path_ = *path;
		}

		ROS_INFO("after CB : current=%d previous=%d", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
	}
}

int Supervisor::cutPath(const nav_msgs::Path& path1, nav_msgs::Path& path2)
{
	float dist = sqrt(pow(path1.poses[0].pose.position.x-human_pose_.x,2) + pow(path1.poses[0].pose.position.y-human_pose_.y,2));
	float dist_min = dist;
	int i_min = 0;
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

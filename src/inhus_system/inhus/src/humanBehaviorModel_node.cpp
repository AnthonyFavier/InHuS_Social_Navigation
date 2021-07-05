#include "humanBehaviorModel.h"


//////////////////// CONFLICT MANAGER /////////////////////

ConflictManager::ConflictManager(ros::NodeHandle nh, bool* want_robot_placed)
: replan_freq_(1)
, blocked_ask_path_freq_(2.0)
, approach_freq_(2.0)
, delay_place_robot_(0.3)
{
	nh_ = nh;
	want_robot_placed_ = want_robot_placed;

	// Ros Params
	ros::NodeHandle private_nh("~"); float f_nb;
	nh_.param(std::string("replan_freq"), f_nb, float(2.0)); replan_freq_ = ros::Rate(f_nb);
	nh_.param(std::string("replan_dist_stop"), replan_dist_stop_, float(0.5));
	private_nh.param(std::string("blocked_ask_path_freq"), f_nb, float(2.0)); blocked_ask_path_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("absolute_path_length_diff"), absolute_path_length_diff_, float(1.0));
	private_nh.param(std::string("ratio_path_length_diff"), ratio_path_length_diff_, float(1.3));
	private_nh.param(std::string("approach_dist"), approach_dist_, float(1.5));
	private_nh.param(std::string("approach_freq"), f_nb, float(2.0)); approach_freq_ = ros::Rate(f_nb);
	ROS_INFO("=> Params ConflictManager :");
	ROS_INFO("replan_freq=%f", 1/replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("replan_dist_stop=%f", replan_dist_stop_);
	ROS_INFO("blocked_ask_path_freq=%f", 1/blocked_ask_path_freq_.expectedCycleTime().toSec());
	ROS_INFO("absolute_path_length_diff=%f", absolute_path_length_diff_);
	ROS_INFO("ratio_path_length_diff=%f", ratio_path_length_diff_);
	ROS_INFO("approach_dist=%f", approach_dist_);
	ROS_INFO("approach_freq=%f", 1/approach_freq_.expectedCycleTime().toSec());

	delay_place_robot_ = ros::Duration(approach_freq_.expectedCycleTime().toSec()/2);

	state_global_ = IDLE;

	srv_get_plan_.request.start.header.frame_id = 	"map";
	srv_get_plan_.request.goal.header.frame_id = 	"map";
	srv_get_plan_.request.tolerance = 		0.1;

	// Publishers
	pub_log_ = nh_.advertise<std_msgs::String>("log", 100);
	pub_cancel_goal_ =	nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_stop_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("stop_cmd", 100);

	// Subscribers
	sub_path_ =	nh_.subscribe("move_base/GlobalPlanner/plan", 100, &ConflictManager::pathCB, this);
	sub_status_move_base_ = nh_.subscribe("move_base/status", 100, &ConflictManager::stateMoveBaseCB, this);

	// Service servers
	server_check_conflict_ =	nh_.advertiseService("check_conflict", &ConflictManager::srvCheckConflict, this);
	server_init_conflict_ = 		nh_.advertiseService("init_check_conflict", &ConflictManager::srvInitCheckConflict, this);

	// Service clients
	ros::service::waitForService("cancel_goal_and_stop");
	client_cancel_goal_and_stop_ = 	nh_.serviceClient<std_srvs::Empty>("cancel_goal_and_stop");
	ros::service::waitForService("move_base/GlobalPlanner/make_plan");
	client_make_plan_ =				nh_.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");
	ros::service::waitForService("update_robot_map");
	client_update_robot_map_ = nh_.serviceClient<std_srvs::Empty>("update_robot_map");
	ros::service::waitForService("resumeSupervisor");
	client_resume_supervisor_ = nh_.serviceClient<std_srvs::Empty>("resumeSupervisor");
}

bool ConflictManager::srvCheckConflict(inhus::ActionBool::Request &req, inhus::ActionBool::Response &res)
{
	res.conflict = 	false;

	// Check NO PATH
	/*switch(goal_status_.status)
	{
		case 0:
			ROS_INFO("CLIENT STATE : PENDING");
			break;
		case 1:
			ROS_INFO("CLIENT STATE : ACTIVE");
			break;
		case 2:
			ROS_INFO("CLIENT STATE : PREEMPTED");
			break;
		case 3:
			ROS_INFO("CLIENT STATE : SUCCEEDED");
			break;
		case 4:
			ROS_INFO("CLIENT STATE : ABORTED");
			break;
		case 5:
			ROS_INFO("CLIENT STATE : REJECTED");
			break;
		case 6:
			ROS_INFO("CLIENT STATE : PREEMPTING");
			break;
		case 7:
			ROS_INFO("CLIENT STATE : RECALLING");
			break;
		case 8:
			ROS_INFO("CLIENT STATE : RECALLED");
			break;
		case 9:
			ROS_INFO("CLIENT STATE : LOST");
			break;
	}*/
	if(goal_status_.status==actionlib::SimpleClientGoalState::ABORTED)
	{
		ROS_INFO("Checked NO_PATH");
		state_blocked_ = NO_PATH;
		res.conflict = 	true;
	}

	// Check if path changed too much
	//ROS_INFO("check : current=%d previous=%d", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
	if((int)previous_path_.poses.size() != 0 && (int)current_path_.poses.size() != 0)
	{
		float current_path_length = computePathLength(&current_path_);
		float previous_path_length = computePathLength(&previous_path_);

		if(abs(current_path_length-previous_path_length) > absolute_path_length_diff_ 	// if difference big enough in absolute
		&& current_path_length > ratio_path_length_diff_*previous_path_length)   		// and if difference big enough relatively
		{
			ROS_INFO("Checked PATH_CHANGED_TOO_MUCH %f, %f", current_path_length, previous_path_length);
			state_blocked_ = LONGER;
			res.conflict = 	true;
		}
	}

	if(res.conflict)
	{
		// remove robot
		ROS_INFO("remove robot in checked");
		*want_robot_placed_ = false;
		client_update_robot_map_.call(srv_signal_);

		// stop human nav goal
		actionlib_msgs::GoalID goal_id;
		pub_cancel_goal_.publish(goal_id);

		// stop human
		pub_stop_cmd_.publish(geometry_msgs::Twist());

		current_action_ = req.action;
		current_path_.poses.clear();
		state_global_ = APPROACH;
		state_approach_ = FIRST;
		ROS_INFO("switch to APPROACH");

		last_replan_ = ros::Time::now() - approach_freq_.expectedCycleTime();
	}

	return true;
}

bool ConflictManager::srvInitCheckConflict(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	//ROS_INFO("Check conflict init");
	current_path_.poses.clear();
	previous_path_.poses.clear();
	return true;
}

void ConflictManager::updateData(geometry_msgs::Pose2D h_pose, geometry_msgs::Twist h_vel, geometry_msgs::Pose2D r_pose, geometry_msgs::Twist r_vel)
{
	h_pose_ = 	h_pose;
	h_vel_  = 	h_vel;
	r_pose_ = 	r_pose;
	r_vel_  = 	r_vel;
}

void ConflictManager::loop()
{
	switch (state_global_)
	{
		case IDLE:
			break;

		case APPROACH:
		{
			msg_.data = "CONFLICT_MANAGER STATE APPROACH " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			// if close enough => BLOCKED
			float dist_to_robot = sqrt(pow(h_pose_.x - r_pose_.x,2) + pow(h_pose_.y - r_pose_.y,2));
			if(dist_to_robot <= approach_dist_)
			{
				ROS_INFO("close enough => switch to BLOCKED");

				// switch to BLOCKED
				state_global_ = BLOCKED;

				// place back robot
				*want_robot_placed_ = true;
				client_update_robot_map_.call(srv_signal_);

				// stop human
				client_cancel_goal_and_stop_.call(srv_signal_);

				last_replan_ = ros::Time::now() - blocked_ask_path_freq_.expectedCycleTime();
				delay_place_robot_.sleep();
			}
			else
			{
				switch(state_approach_)
				{
					case FIRST:
						delay_place_robot_.sleep();
						state_approach_ = REPLANNING;
						break;

					// Replan without the robot
					case REPLANNING:
						if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
						{
							ROS_INFO("\t => APPROACH <=");
							ROS_INFO("REPLANNING");
							ROS_INFO("dist=%f", dist_to_robot);

							// if not too close from approach_dist
							if(dist_to_robot > approach_dist_ + replan_dist_stop_)
							{
								move_base_msgs::MoveBaseActionGoal goal;
								goal.goal.target_pose.header.frame_id = "map";
								goal.goal.target_pose.header.stamp = ros::Time::now();
								goal.goal = current_action_;
								pub_goal_move_base_.publish(goal);
							}
							state_approach_ = PLACE_ROBOT;
							last_replan_ = ros::Time::now();
						}
						break;

					// Place the robot back for next checking with it
					case PLACE_ROBOT:
						if(ros::Time::now() - last_replan_ > delay_place_robot_)
						{
							// put the robot back on the map to check if still blocked in next approach loop
							*want_robot_placed_ = true;
							client_update_robot_map_.call(srv_signal_);
							ROS_INFO("put robot in replanning (for check)");
							state_approach_ = CHECKING;
							last_replan_ = ros::Time::now();
						}
						break;

					// check, with the robot, if the human is still blocked
					case CHECKING:
						if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
						{
							ROS_INFO("\t => APPROACH <=");
							ROS_INFO("CHECKING");
							ROS_INFO("dist=%f", dist_to_robot);

							// check if still blocked
							bool still_blocked = true;
							srv_get_plan_.request.start.pose.position.x = 	h_pose_.x;
							srv_get_plan_.request.start.pose.position.y = 	h_pose_.y;
							srv_get_plan_.request.goal.pose.position.x = 	current_action_.target_pose.pose.position.x;
							srv_get_plan_.request.goal.pose.position.y = 	current_action_.target_pose.pose.position.y;
							if(client_make_plan_.call(srv_get_plan_))
							{
								//ROS_INFO("make_plan %d", (int)srv_get_plan_.response.plan.poses.size());
								if((int)srv_get_plan_.response.plan.poses.size()!=0) // path found
								{
									float response_path_length = computePathLength(&(srv_get_plan_.response.plan));
									float previous_path_length = computePathLength(&previous_path_);
									ROS_INFO("check path length = %f", response_path_length);
									ROS_INFO("check previous path length = %f", previous_path_length);

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
								last_replan_ = ros::Time::now();
								state_approach_ = REMOVE_ROBOT;
							}
							else
							{
								// since the human isn't blocked anymore, switch back to EXEC_PLAN
								ROS_INFO("Not BLOCKED while approaching back to IDLE");
								state_global_ = IDLE;
								client_resume_supervisor_.call(srv_signal_);
							}
						}
						break;

					// Remove the robot for next replan without it
					case REMOVE_ROBOT:
						if(ros::Time::now() - last_replan_ > delay_place_robot_)
						{
							// remove the robot from the map for replanning in next approach loop
							*want_robot_placed_ = false;
							client_update_robot_map_.call(srv_signal_);
							ROS_INFO("remove robot in checking (for replan)");
							last_replan_ = ros::Time::now();
							state_approach_ = REPLANNING;
						}
						break;
				}
			}
			break;
		}

		case BLOCKED:
			msg_.data = "CONFLICT_MANAGER STATE BLOCKED " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_);

			// back to APPROACH if too far
			float dist_to_robot = sqrt(pow(h_pose_.x - r_pose_.x,2) + pow(h_pose_.y - r_pose_.y,2));
			//ROS_INFO("dist=%f", dist_to_robot);
			if(dist_to_robot > approach_dist_)
			{
				ROS_INFO("Back to APPROACH");

				// remove robot
				*want_robot_placed_ = true;
				client_update_robot_map_.call(srv_signal_);

				last_replan_ = ros::Time::now() - approach_freq_.expectedCycleTime();

				// switch to APPROACH
				state_global_ = APPROACH;
				state_approach_ = FIRST;
			}
			else if(ros::Time::now() - last_replan_ > blocked_ask_path_freq_.expectedCycleTime())
			{
				ROS_INFO("\t => BLOCKED <=");
				//ROS_INFO("try to replan");

				srv_get_plan_.request.start.pose.position.x = 	h_pose_.x;
				srv_get_plan_.request.start.pose.position.y = 	h_pose_.y;
				srv_get_plan_.request.goal.pose.position.x = 	current_action_.target_pose.pose.position.x;
				srv_get_plan_.request.goal.pose.position.y = 	current_action_.target_pose.pose.position.y;

				// make plan
				if(client_make_plan_.call(srv_get_plan_))
				{
					//ROS_INFO("BLOCK : srv.plan=%d previous=%d", (int)srv_get_plan_.response.plan.poses.size(), (int)previous_path_.poses.size());
					last_replan_ = ros::Time::now();
					if(!srv_get_plan_.response.plan.poses.empty()) // successfully planned once
					{
						float response_path_length = computePathLength(&(srv_get_plan_.response.plan));
						float previous_path_length = computePathLength(&previous_path_);

						ROS_INFO("blocked path length = %f", response_path_length);
						ROS_INFO("blocked previous path length = %f", previous_path_length);

						// If new path is 'good'
						if(previous_path_length == 0								// if no path was found before
						|| abs(response_path_length-previous_path_length)<absolute_path_length_diff_		// if close enough in absolute
						|| response_path_length < ratio_path_length_diff_*previous_path_length)   		// or if close enough relatively
						{
							ROS_INFO("Not BLOCKED back to IDLE");
							state_global_ = IDLE;
							client_resume_supervisor_.call(srv_signal_);
						}
					}
				}
				else
					ROS_ERROR("Failed to call service make_plan");
			}
			break;
	}
}

void ConflictManager::pathCB(const nav_msgs::Path::ConstPtr& path)
{
	//ROS_INFO("path CB=%d curr=%d prev=%d", (int)path->poses.size(), (int)current_path_.poses.size(), (int)previous_path_.poses.size());

	float path_length = computePathLength(path.get());

	std_msgs::String msg;
	msg.data = "CONFLICT_MANAGER PATH " + std::to_string(path_length) + " " + std::to_string(path->header.stamp.toSec());
	pub_log_.publish(msg);

	if(state_global_ != BLOCKED && state_global_ != APPROACH)
	{
		//ROS_INFO("pathCallback %d length %f ! current %d previous %d", (int)path->poses.size(), path_length,(int)current_path_.poses.size(), (int)previous_path_.poses.size());

		// in order to always have a valid path stored in previous_path_
		// the current_path is always uptaded but the previous is only
		// updated if the current one wasn't empty

		if((int)previous_path_.poses.size()==0)
		{
			//ROS_INFO("FIRST !");
			// first w/o robot
			previous_path_ = *path;
			msg.data = "CONFLICT_MANAGER FIRST " + std::to_string(path_length) + " " + std::to_string(path->header.stamp.toSec());
			pub_log_.publish(msg);
		}
		else
		{
			if((int)current_path_.poses.size()!=0)
			{
				// seek pose closest to current_pose from current_path
				// only keep path from current_pose to the end store to previous
				cutPath(current_path_, previous_path_, h_pose_);
			}
		}
		// update current_path
		current_path_ = *path;

		//ROS_INFO("after CB : current=%d previous=%d", (int)current_path_.poses.size(), (int)previous_path_.poses.size());
	}
}

void ConflictManager::stateMoveBaseCB(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
	if(!status->status_list.empty())
	{
		goal_status_.status = status->status_list.back().status;
	}
}

///////////////////////////////////////////////////////////

/////////////////////// HUMAN MODEL ///////////////////////

HumanBehaviorModel::HumanBehaviorModel(ros::NodeHandle nh)
: check_see_robot_freq_(1)
, b_random_try_freq_(1)
, b_stop_look_stop_dur_(1)
, b_harass_replan_freq_(1)
{
	nh_ = nh;

	srand(time(NULL));

	// Ros Params
	ros::NodeHandle private_nh("~"); float f_nb; int fov_int;
	private_nh.param(std::string("ratio_perturbation_cmd"), ratio_perturbation_cmd_, float(0.0));
	private_nh.param(std::string("fov"), fov_int, int(180)); fov_ = fov_int*PI/180;
	private_nh.param(std::string("check_see_robot_freq"), f_nb, float(5.0)); check_see_robot_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("delay_forget_robot"), f_nb, float(1.5)); delay_forget_robot_ = ros::Duration(f_nb);
	private_nh.param(std::string("human_radius"), human_radius_, float(0.25));
	private_nh.param(std::string("robot_radius"), robot_radius_, float(0.3));
	private_nh.param(std::string("b_random_chance_choose"), b_random_chance_choose_, int(30));
	private_nh.param(std::string("b_random_try_freq"), f_nb, float(0.5)); b_random_try_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("b_stop_look_dist_near_robot"), b_stop_look_dist_near_robot_, float(2.0));
	private_nh.param(std::string("b_stop_look_stop_dur"), f_nb, float(2.0)); b_stop_look_stop_dur_ = ros::Duration(f_nb);
	private_nh.param(std::string("b_harass_dist_in_front"), b_harass_dist_in_front_, float(2.0));
	private_nh.param(std::string("b_harass_replan_freq"), f_nb, float(2.0)); b_harass_replan_freq_ = ros::Rate(f_nb);
	ROS_INFO("=> Params HBM:");
	ROS_INFO("ratio_perturbation_cmd=%f", ratio_perturbation_cmd_);
	ROS_INFO("fov_int=%d fov=%f", fov_int, fov_);
	ROS_INFO("check_see_robot_freq=%f", 1/check_see_robot_freq_.expectedCycleTime().toSec());
	ROS_INFO("delay_forget_robot=%f", delay_forget_robot_.toSec());
	ROS_INFO("human_radius=%f", human_radius_);
	ROS_INFO("robot_radius=%f", robot_radius_);
	ROS_INFO("b_random_chance_choose=%d", b_random_chance_choose_);
	ROS_INFO("b_random_try_freq=%f", 1/b_random_try_freq_.expectedCycleTime().toSec());
	ROS_INFO("b_stop_look_dist_near_robot=%f", b_stop_look_dist_near_robot_);
	ROS_INFO("b_stop_look_stop_dur=%f", b_stop_look_stop_dur_.toSec());
	ROS_INFO("b_harass_dist_in_front=%f", b_harass_dist_in_front_);
	ROS_INFO("b_harass_replan_freq=%f", 1/b_harass_replan_freq_.expectedCycleTime().toSec());

	// Subscribers
	sub_pose_ = 	 	nh_.subscribe("interface/in/human_pose", 100, &HumanBehaviorModel::poseCallback, this);
	sub_vel_ = 	 	nh_.subscribe("interface/in/human_vel", 100, &HumanBehaviorModel::velCallback, this);
	sub_robot_pose_ =	nh_.subscribe("interface/in/robot_pose", 100, &HumanBehaviorModel::robotPoseCallback, this);
	sub_robot_vel_ =	nh_.subscribe("interface/in/robot_vel", 100, &HumanBehaviorModel::robotVelCallback, this);
	sub_cmd_geo_ =		nh_.subscribe("cmd_geo", 100, &HumanBehaviorModel::cmdGeoCallback, this);
	sub_goal_done_ =	nh_.subscribe("goal_done", 100, &HumanBehaviorModel::goalDoneCallback, this);
	sub_set_attitude_ = 	nh_.subscribe("/boss/human/set_attitude", 100, &HumanBehaviorModel::setAttitudeCallback, this);
	sub_new_goal_ =		nh_.subscribe("/boss/human/new_goal", 100, &HumanBehaviorModel::newGoalCallback, this);
	sub_stop_cmd_ = 	nh_.subscribe("stop_cmd", 100, &HumanBehaviorModel::stopCmdCallback, this);
	sub_pov_map_ = 		nh_.subscribe("map_pov", 1, &HumanBehaviorModel::povMapCallback, this);

	// Publishers
	pub_human_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("known/human_pose", 100);
	pub_human_vel_ = 	nh_.advertise<geometry_msgs::Twist>("known/human_vel", 100);
	pub_robot_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("known/robot_pose", 100);
	pub_robot_vel_ = 	nh_.advertise<geometry_msgs::Twist>("known/robot_vel", 100);
	pub_new_goal_ = 	nh_.advertise<inhus::Goal>("new_goal", 100);
	pub_perturbed_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("perturbed_cmd", 100);
	pub_goal_move_base_ =	nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_log_ = 		nh_.advertise<std_msgs::String>("log", 100);

	// Service clients
	ros::service::waitForService("set_wait_goal");
	client_set_wait_goal_ = 		nh_.serviceClient<std_srvs::Empty>("set_wait_goal");
	ros::service::waitForService("cancel_goal_and_stop");
	client_cancel_goal_and_stop_ = 	nh_.serviceClient<std_srvs::Empty>("cancel_goal_and_stop");
	ros::service::waitForService("place_robot");
	client_place_robot_ = 		nh_.serviceClient<inhus_navigation::PlaceRobot>("place_robot");
	ros::service::waitForService("suspendSupervisor");
	client_suspend_supervisor_ = nh_.serviceClient<std_srvs::Empty>("suspendSupervisor");
	ros::service::waitForService("resumeSupervisor");
	client_resume_supervisor_ = nh_.serviceClient<std_srvs::Empty>("resumeSupervisor");

	// Service server
	server_place_robot_ = nh_.advertiseService("place_robot_hm", &HumanBehaviorModel::srvPlaceRobotHM, this);
	server_update_robot_map_ = nh_.advertiseService("update_robot_map", &HumanBehaviorModel::srvUpdateRobotMap, this);

	//ROS_INFO("I am human");

	// Init
	geometry_msgs::Pose2D zero;
	zero.x = 	0;
	zero.y = 	0;
	zero.theta = 	0;
	sim_pose_ =		zero;
	sim_robot_pose_=    	zero;
	model_pose_ = 	   	zero;
	model_robot_pose_ =	zero;

	current_goal_.type = 	"navigation";
	current_goal_.x  =	0;
	current_goal_.y = 	0;
	current_goal_.theta = 	0;

	previous_goal_=current_goal_;

	executing_plan_ = 	false;

	last_check_see_robot_ = ros::Time::now();
	last_seen_robot_ = 	ros::Time::now();
	last_time_ = 		ros::Time::now();
	last_harass_ = 		ros::Time::now();
	time_stopped_=		ros::Time::now();

	see_ = false;

	radius_sum_sq_ = human_radius_ + robot_radius_;
	radius_sum_sq_ *= radius_sum_sq_;
	ttc_ = -1.0;

	hcb_ = 		false;
	rcb_ = 		false;

	pmcb_ = false;
	know_robot_pose_ = false;
	want_robot_placed_ = true;

	// ATTITUDES //
	attitude_ = NONE;
	sub_stop_look_ = WAIT_ROBOT;
	sub_harass_ = INIT;

	// INIT GOALS
	goal_file_name_ = "goals.xml";
	std::string goal_file_path = ros::package::getPath("inhus") + "/config/" + goal_file_name_;
	doc_ = new TiXmlDocument(goal_file_path);
	if(!doc_->LoadFile())
		ROS_ERROR("Failed to load %s", goal_file_path.c_str());
	else
		ROS_INFO("HBM: Goals file loaded");
	this->readGoalsFromXML();
	// this->showGoals();
}

void HumanBehaviorModel::readGoalsFromXML()
{
	TiXmlHandle docHandle(doc_);
	GoalArea area;

	// Extracting the list of goals
	TiXmlElement* l_goal = docHandle.FirstChild("goals").FirstChild("goal_list").FirstChild("goal").ToElement();
	while(l_goal)
	{
		if(NULL != l_goal->Attribute("type"))
			area.goal.type = l_goal->Attribute("type");
		if(area.goal.type == "navigation")
		{
			if(NULL != l_goal->Attribute("x"))
				area.goal.x = std::stof(l_goal->Attribute("x"));
			if(NULL != l_goal->Attribute("y"))
				area.goal.y = std::stof(l_goal->Attribute("y"));
			if(NULL != l_goal->Attribute("theta"))
				area.goal.theta = std::stof(l_goal->Attribute("theta"));
			if(NULL != l_goal->Attribute("radius"))
				area.radius = std::stof(l_goal->Attribute("radius"));
		}
		known_goals_.push_back(area);

		l_goal = l_goal->NextSiblingElement("goal");
	}
}

void HumanBehaviorModel::showGoals()
{
	// list goals
	std::cout << "=> list_goals <=" << std::endl;
	for(unsigned int i=0; i<known_goals_.size(); i++)
		std::cout << "\t" << known_goals_[i].goal.type << " " << known_goals_[i].goal.x << " " << known_goals_[i].goal.y << " " << known_goals_[i].goal.theta << " " << known_goals_[i].radius << std::endl;
}

void HumanBehaviorModel::publishGoal(GoalArea goal)
{
	goal = computeGoalWithRadius(goal);
	executing_plan_ = true;
	pub_new_goal_.publish(goal.goal);
}

void HumanBehaviorModel::processSimData()
{
	model_pose_ = 		sim_pose_;
	model_vel_ = 		sim_vel_;
	model_robot_pose_ = 	sim_robot_pose_;
	model_robot_vel_ = 	sim_robot_vel_;
}

void HumanBehaviorModel::publishModelData()
{
	pub_human_pose_.publish(model_pose_);
	pub_human_vel_.publish(model_vel_);
	pub_robot_pose_.publish(model_robot_pose_);
	pub_robot_vel_.publish(model_robot_vel_);
}

void HumanBehaviorModel::pubDist()
{
	float dist = sqrt(pow(model_robot_pose_.x-model_pose_.x,2) + pow(model_robot_pose_.y-model_pose_.y,2));
	msg_log_.data = "HUMAN_MODEL DIST " + std::to_string(dist) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg_log_);
}

void HumanBehaviorModel::computeTTC()
{
	ttc_ = -1.0; // ttc infinite

	geometry_msgs::Pose2D C; // robot to human distance
	C.x = model_pose_.x - model_robot_pose_.x;
	C.y = model_pose_.y - model_robot_pose_.y;
	double C_sq = C.x*C.x + C.y*C.y; // dot product C.C

	if(C_sq <= radius_sum_sq_) // already touching
		ttc_ = 0.0;
	else
	{
		geometry_msgs::Twist V; // relative velocity human to robot
		V.linear.x = model_robot_vel_.linear.x - model_vel_.linear.x;
		V.linear.y = model_robot_vel_.linear.y - model_vel_.linear.y;

		//V.linear.x = model_vel_.linear.x - model_robot_vel_.linear.x;
		//V.linear.y = model_vel_.linear.y - model_robot_vel_.linear.y;
		double C_dot_V = C.x*V.linear.x + C.y*V.linear.y;

		if(C_dot_V > 0) // otherwise ttc infinite
		{
			double V_sq = V.linear.x*V.linear.x + V.linear.y*V.linear.y;
			double f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_));
			if(f > 0) // otherwise ttc infinite
				ttc_ = (C_dot_V - sqrt(f)) / V_sq;
		}
	}

	if(ttc_ != -1)
	{
	//	ROS_INFO("TTC = %f", ttc_);
		msg_log_.data = "HUMAN_MODEL TTC " + std::to_string(ttc_) + " " + std::to_string(ros::Time::now().toSec());
		pub_log_.publish(msg_log_);
	}
}

void HumanBehaviorModel::computeRelSpd()
{
	relative_speed_ = sqrt(pow(model_vel_.linear.x-model_robot_vel_.linear.x,2) + pow(model_vel_.linear.y-model_robot_vel_.linear.y,2));
	msg_log_.data = "HUMAN_MODEL REL_SPD " + std::to_string(relative_speed_) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg_log_);
}

bool HumanBehaviorModel::initDone()
{
	return hcb_ && rcb_ && pmcb_;
}

///////////////////// Visibility //////////////////////////

bool HumanBehaviorModel::testObstacleView(geometry_msgs::Pose2D A_real, geometry_msgs::Pose2D B_real)
{
	// check if there are obstacles preventing A from seeing B

	int A_map_x; int A_map_y;
	A_map_x = (int)(A_real.x / resol_pov_map_); A_map_y = (int)(A_real.y / resol_pov_map_);
	int B_map_x; int B_map_y;
	B_map_x = (int)(B_real.x / resol_pov_map_); B_map_y = (int)(B_real.y / resol_pov_map_);

	// if outside the map
	if(A_map_x < 0 || A_map_x >= g_map_[0].size() || A_map_y < 0 || A_map_x >= g_map_.size()
	|| B_map_x < 0 || B_map_x >= g_map_[0].size() || B_map_y < 0 || B_map_x >= g_map_.size())
		return false;

	// particular cases
	// if one of the poses is an obstacle
	if(g_map_[A_map_y][A_map_x] == 1 || g_map_[B_map_y][B_map_x] == 1)
		return false;
	else if(A_map_x == B_map_x || A_map_y == B_map_y)
	{
		// same place
		if(A_map_x == B_map_x && A_map_y == B_map_y)
			return true;

		// vertical
		else if(A_map_x == B_map_x)
		{
			for(int i=0; A_map_y + i != B_map_y;)
			{
				int xi = A_map_x;
				int yi = A_map_y + i;

				if(g_map_[yi][xi]==1) // if obstacle
					return false;

				// up
				if(B_map_y > A_map_y)
					i++;
				// down
				else
					i--;
			}
		}

		// horizontal
		else if(A_map_y == B_map_y)
		{
			for(int i=0; A_map_x + i != B_map_x;)
			{
				int xi = A_map_x + i;
				int yi = A_map_y;

				if(g_map_[yi][xi]==1)
					return false;

				// right
				if(B_map_x > A_map_x)
					i++;
				// left
				else
					i--;
			}
		}
	}
	// general cases
	else
	{
		float m = (float)(B_map_y - A_map_y)/(float)(B_map_x - A_map_x);
		float b = A_map_y - m * A_map_x;

		float marge = 0.9;
		float delta_x = std::min(marge/abs(m), marge);

		// sign
		if(B_map_x < A_map_x)
			delta_x = -delta_x;

		int i=1;
		bool cond = true;
		while(cond)
		{
			float xi_f = A_map_x + i * delta_x;
			float yi_f = m * xi_f + b;

			int xi = (int)(xi_f);
			int yi = (int)(yi_f);

			if(g_map_[yi][xi]==1) // if obstacle
				return false;

			i++;
			if(delta_x > 0)
				cond = i*delta_x + A_map_x < B_map_x;
			else
				cond = i*delta_x + A_map_x > B_map_x;
		}
	}

	return true;
}

bool HumanBehaviorModel::testFOV(geometry_msgs::Pose2D A, geometry_msgs::Pose2D B, float fov)
{
	// check if A is in the specified field of view of B (w/o obstacle)
	float alpha;
	float qy = A.y - B.y;
	float qx = A.x - B.x;
	if(qx==0)
	{
		if(qy>0)
			alpha = PI/2;
		else
			alpha = -PI/2;
	}
	else
	{
		float q = abs(qy/qx);
		if(qx>0)
		{
			if(qy>0)
				alpha = atan(q);
			else
				alpha = -atan(q);
		}
		else
		{
			if(qy>0)
				alpha = PI - atan(q);
			else
				alpha = atan(q) - PI;
		}
	}

	float diff;
	if(alpha * B.theta > 0) // same sign
		diff = abs(alpha - B.theta);
	else
	{
		if(alpha < 0)
			diff = std::min(abs(alpha - B.theta), abs((alpha+2*PI) - B.theta));
		else
			diff = std::min(abs(alpha - B.theta), abs(alpha - (B.theta+2*PI)));
	}

	return diff < fov/2;
}

void HumanBehaviorModel::updateRobotOnMap()
{
	if(see_)
	{
		if(want_robot_placed_)
		{
			if(!know_robot_pose_) // rising edge
			{
				//ROS_INFO("place_robot true");
				know_robot_pose_ = true;
				srv_place_robot_.request.data = true;
				client_place_robot_.call(srv_place_robot_);
			}
		}
		else
		{
			if(know_robot_pose_) // falling edge
			{
				//ROS_INFO("place_robot false");
				know_robot_pose_ = false;
				srv_place_robot_.request.data = false;
				client_place_robot_.call(srv_place_robot_);
			}

		}


	}
	else
	{
		if(want_robot_placed_)
		{
			if(ros::Time::now() - last_seen_robot_ > ros::Duration(1.5)) // delay see robot, memory/prediction
			{
				if(know_robot_pose_) // falling edge
				{
					//ROS_INFO("place_robot false");
					know_robot_pose_ = false;
					srv_place_robot_.request.data = false;
					client_place_robot_.call(srv_place_robot_);
				}
			}
		}
		else
		{
			if(know_robot_pose_) // falling edge
			{
				//ROS_INFO("place_robot false");
				know_robot_pose_ = false;
				srv_place_robot_.request.data = false;
				client_place_robot_.call(srv_place_robot_);
			}

		}
	}
}

void HumanBehaviorModel::testSeeRobot()
{
	if(ros::Time::now() - last_check_see_robot_ > check_see_robot_freq_.expectedCycleTime())
	{
		geometry_msgs::Pose2D human_pose_offset = model_pose_;
		human_pose_offset.x -= offset_pov_map_x_;
		human_pose_offset.y -= offset_pov_map_y_;
		geometry_msgs::Pose2D robot_pose_offset = model_robot_pose_;
		robot_pose_offset.x -= offset_pov_map_x_;
		robot_pose_offset.y -= offset_pov_map_y_;

		// check if the robot is in the field of view of the human
		// (without obstacles)
		if(this->testFOV(robot_pose_offset, human_pose_offset, fov_))
		{
			// check if there are obstacles blocking the human view of the robot
			if(this->testObstacleView(human_pose_offset, robot_pose_offset))
			{
				// the human sees the robot
				//ROS_INFO("I SEE");
				see_ = true;
				last_seen_robot_ = ros::Time::now();
			}
			else
			{
				// human can't see the robot
				//ROS_INFO("VIEW IS BLOCKED");
				see_ = false;
			}
		}
		else
		{
			//ROS_INFO("NOT IN FOV");
			see_ = false;
		}

		// Update robot on map if needed
		this->updateRobotOnMap();

		last_check_see_robot_ = ros::Time::now();
	}
}

////////////////////// Attitudes //////////////////////////

GoalArea HumanBehaviorModel::chooseGoal(bool random)
{
	GoalArea goal;
	static int index_list=-1;
	int i=0;

	if(random)
	{
		do
		{
			i=rand()%known_goals_.size();
		}while(known_goals_[i].goal.x==previous_goal_.x && known_goals_[i].goal.y==previous_goal_.y);
	}
	// follow list of known goals
	else
	{
		index_list=(index_list+1)%known_goals_.size();
		i = index_list;
	}

	// if it's an area, pick a goal in it
	goal = computeGoalWithRadius(known_goals_[i]);

	current_goal_=goal.goal;

	return goal;
}

void HumanBehaviorModel::attNonStop()
{
	if(!executing_plan_)
	{
		GoalArea goal = chooseGoal(true);

		this->publishGoal(goal);
	}
}

void HumanBehaviorModel::attRandom()
{
	if(ros::Time::now()-last_time_> b_random_try_freq_.expectedCycleTime())
	{
		int nb = rand()%100 + 1;
		//ROS_INFO("Tirage %d/%d", nb, b_random_chance_choose_);
		if(nb < b_random_chance_choose_)
		{
			//ROS_INFO("DECIDE NEW GOAL ! ");
			inhus::Goal previous_goal = current_goal_;
			GoalArea new_goal = this->chooseGoal(true);
			if(new_goal.goal.x != previous_goal.x || new_goal.goal.y != previous_goal.y)
				this->publishGoal(new_goal);
		}
		last_time_=ros::Time::now();
	}
}

void HumanBehaviorModel::attStopLook()
{
	switch(sub_stop_look_)
	{
		case WAIT_ROBOT:
			{
				float dist = sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2));
				//ROS_INFO("threshold=%f dist=%f", b_stop_look_dist_near_robot_, dist);
				if(dist<b_stop_look_dist_near_robot_)
					sub_stop_look_=STOP;
				break;
			}

		case STOP:
			{
				// Suspend the supervisor
				client_suspend_supervisor_.call(srv_signal_);

				// Stop goal and motion
				client_cancel_goal_and_stop_.call(srv_signal_);

				// Get time
				time_stopped_ = ros::Time::now();

				sub_stop_look_=LOOK_AT_ROBOT;
				break;
			}

		case LOOK_AT_ROBOT:
				if(ros::Time::now() - time_stopped_ > b_stop_look_stop_dur_)
					sub_stop_look_=RESUME_GOAL;
				else
				{
					float qy = model_robot_pose_.y - model_pose_.y;
					float qx = model_robot_pose_.x - model_pose_.x;

					float q;
					float alpha;

					if(qx==0)
					{
						if(qy>0)
							alpha = PI/2;
						else
							alpha = -PI/2;
					}
					else
					{
						q = abs(qy/qx);
						if(qx>0)
						{
							if(qy>0)
								alpha = atan(q);
							else
								alpha = -atan(q);
						}
						else
						{
							if(qy>0)
								alpha = PI - atan(q);
							else
								alpha = atan(q) - PI;
						}
					}

					geometry_msgs::Twist cmd;
					if(abs(alpha-model_pose_.theta)>0.1)
					{
						cmd.angular.z=2;
						if(alpha-model_pose_.theta<0)
							cmd.angular.z=-cmd.angular.z;

						if(abs(alpha-model_pose_.theta)>PI)
							cmd.angular.z=-cmd.angular.z;
					}
					pub_perturbed_cmd_.publish(cmd);
				}
				break;

		case RESUME_GOAL:
			client_resume_supervisor_.call(srv_signal_);
			sub_stop_look_=OVER;
			break;

		case OVER:
			{
				// Wait for robot to get far enough to reset
				float dist = sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2));
				if(dist>b_stop_look_dist_near_robot_)
				{
					//ROS_INFO("Reset STOP_LOOK");
					sub_stop_look_=WAIT_ROBOT;
				}
				break;
			}

		default:
			sub_stop_look_=WAIT_ROBOT;
			break;
	}
}

void HumanBehaviorModel::attHarass()
{
	switch(sub_harass_)
	{
		case INIT:
			{
				// suspend cancel stop
				client_set_wait_goal_.call(srv_signal_);
				client_cancel_goal_and_stop_.call(srv_signal_);
				sub_harass_=HARASSING;
				break;
			}
		case HARASSING:
			if(ros::Time::now() > last_harass_ + b_harass_replan_freq_.expectedCycleTime())
			{
				geometry_msgs::Pose2D in_front;
				in_front.x = model_robot_pose_.x + cos(model_robot_pose_.theta)*b_harass_dist_in_front_;
				in_front.y = model_robot_pose_.y + sin(model_robot_pose_.theta)*b_harass_dist_in_front_;
				in_front.theta = model_robot_pose_.theta;

				move_base_msgs::MoveBaseActionGoal goal;
				goal.goal.target_pose.header.frame_id = "map";
				goal.goal.target_pose.header.stamp = ros::Time::now();
				goal.goal.target_pose.pose.position.x = in_front.x;
				goal.goal.target_pose.pose.position.y = in_front.y;
				tf2::Quaternion q;
				q.setRPY(0,0,in_front.theta);
				goal.goal.target_pose.pose.orientation.x = q.x();
				goal.goal.target_pose.pose.orientation.y = q.y();
				goal.goal.target_pose.pose.orientation.z = q.z();
				goal.goal.target_pose.pose.orientation.w = q.w();
				pub_goal_move_base_.publish(goal);

				last_harass_ = ros::Time::now();
			}
			break;

		default:
			sub_harass_=INIT;
			break;
	}
}

void HumanBehaviorModel::attitudes()
{
	switch(attitude_)
	{
		case NONE:
			break;

		case NON_STOP:
			this->attNonStop();
			break;

		case RANDOM:
			this->attRandom();
			break;

		case STOP_LOOK:
			this->attStopLook();
			break;

		case HARASS:
			this->attHarass();
			break;

		default:
			attitude_=NONE;
			break;
	}
}

/////////////////// ConflictManager ///////////////////////

void HumanBehaviorModel::initConflictManager(ConflictManager* conflict_manager)
{
	conflict_manager_ = conflict_manager;
}

void HumanBehaviorModel::updateConflictManager()
{
	conflict_manager_->updateData(sim_pose_, sim_vel_, sim_robot_pose_, sim_robot_vel_);
}

void HumanBehaviorModel::conflictManagerLoop()
{
	conflict_manager_->loop();
}

/////////////////// Service servers ///////////////////////

bool HumanBehaviorModel::srvUpdateRobotMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
	this->updateRobotOnMap();
	return true;
}

bool HumanBehaviorModel::srvPlaceRobotHM(inhus_navigation::PlaceRobot::Request& req, inhus_navigation::PlaceRobot::Response& res)
{
	want_robot_placed_ = req.data;

	return true;
}

////////////////////// Callbacks //////////////////////////

void HumanBehaviorModel::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_pose_.x=msg->x;
	sim_pose_.y=msg->y;
	sim_pose_.theta=msg->theta;

	if(!hcb_)
	{
		//ROS_INFO("hcb");
		hcb_=true;
	}
}

void HumanBehaviorModel::velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	sim_vel_ = *msg;
}

void HumanBehaviorModel::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_robot_pose_.x=msg->x;
	sim_robot_pose_.y=msg->y;
	sim_robot_pose_.theta=msg->theta;

	if(!rcb_)
	{
		//ROS_INFO("rcb");
		rcb_=true;
	}
}

void HumanBehaviorModel::robotVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	sim_robot_vel_ = *msg;
}

void HumanBehaviorModel::cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist perturbed_cmd;

	perturbed_cmd.linear.x = msg->linear.x * (1 + ratio_perturbation_cmd_*((float)(rand()%300-100)/100.0));
	perturbed_cmd.linear.y = msg->linear.y * (1 + ratio_perturbation_cmd_*((float)(rand()%300-100)/100.0));
	perturbed_cmd.linear.z = 0;
	perturbed_cmd.angular.x = 0;
	perturbed_cmd.angular.y = 0;
	perturbed_cmd.angular.z = msg->angular.z; // no angular perturbation

	pub_perturbed_cmd_.publish(perturbed_cmd);
}

void HumanBehaviorModel::stopCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	if((attitude_ != HARASS) && (attitude_ != STOP_LOOK || sub_stop_look_ != LOOK_AT_ROBOT))
		pub_perturbed_cmd_.publish(*cmd);
}

void HumanBehaviorModel::goalDoneCallback(const inhus::Goal::ConstPtr& msg)
{
	//ROS_INFO("goal done !!!!!!!");
	previous_goal_ = current_goal_;
	executing_plan_ = false;
}

void HumanBehaviorModel::newGoalCallback(const inhus::Goal::ConstPtr& goal)
{
	previous_goal_ = 	current_goal_;
	current_goal_ = 	*goal;

	GoalArea goal_area;
	goal_area.goal = *goal;
	goal_area.radius = 0.0;
	this->publishGoal(goal_area);
}

void HumanBehaviorModel::setAttitudeCallback(const std_msgs::Int32::ConstPtr& msg)
{
	bool changed=false;
	switch(msg->data)
	{
		case NONE: //0
			ROS_INFO("Attitude set : NONE");
			changed=true;
			break;
		case NON_STOP: //1
			ROS_INFO("Attitude set : NON_STOP");
			changed=true;
			break;
		case RANDOM: //2
			ROS_INFO("Attitude set : RANDOM");
			changed=true;
			break;
		case STOP_LOOK: //3
			ROS_INFO("Attitude set : STOP_LOOK");
			changed=true;
			break;
		case HARASS: //4
			ROS_INFO("Attitude set : HARASS");
			changed=true;
			break;

		default:
			break;
	}

	if(changed)
	{
		attitude_ = (Attitude)msg->data;
		sub_stop_look_ = WAIT_ROBOT;
		sub_harass_ = INIT;
	}
}

void HumanBehaviorModel::povMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	offset_pov_map_x_ = map->info.origin.position.x;
	offset_pov_map_y_ = map->info.origin.position.y;

	int width = map->info.width;
	int height = map->info.height;
	int cell;

	resol_pov_map_ = map->info.resolution;

	for(int i=0; i<height; i++)
	{
		std::vector<int> line;
		for(int j=0; j<width; j++)
		{
			cell = map->data[width*i+j];
			if(cell == 0)
				line.push_back(0);
			else
				line.push_back(1);
		}
		g_map_.push_back(line);
	}

	if(!pmcb_)
	{
		//ROS_INFO("pmcb");
		pmcb_ = true;
	}
}

////////////////////////// MAIN ///////////////////////////

void threadSpin()
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	ros::NodeHandle nh;

	HumanBehaviorModel human_model(nh);
	ConflictManager conflict_manager(nh, &human_model.want_robot_placed_);
	human_model.initConflictManager(&conflict_manager);

	// spawn thread ros spin
	boost::thread thread_a(threadSpin);

	ros::Rate rate(30);

	ROS_INFO("HBM: Waiting for init ... (navigation and localization)");
	while(ros::ok() && !human_model.initDone())
	{
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("HBM: Ready");

	while(ros::ok())
	{
		/* DATA */
		// Process data from simu
		human_model.processSimData();

		// Publish data as perceived by the human model
		human_model.publishModelData();

		// Update robot pose knowledge
		human_model.testSeeRobot();

		// Update data of Conflict Manager
		human_model.updateConflictManager();

		/* BEHAVIOR */
		// Conflict Manager
		human_model.conflictManagerLoop();

		// Attitudes
		human_model.attitudes();

		/* LOGS */
		// Compute TTC
		human_model.computeTTC();

		// Compute relative speed
		human_model.computeRelSpd();

		// Publish human/robot distance to log
		human_model.pubDist();

		ros::spinOnce();
		rate.sleep();
	}
}

///////////////////////////////////////////////////////////

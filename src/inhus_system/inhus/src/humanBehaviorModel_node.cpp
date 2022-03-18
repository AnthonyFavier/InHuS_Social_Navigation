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
	ROS_INFO("CM: => Params ConflictManager :");
	ROS_INFO("CM: replan_freq=%f", 1/replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("CM: replan_dist_stop=%f", replan_dist_stop_);
	ROS_INFO("CM: blocked_ask_path_freq=%f", 1/blocked_ask_path_freq_.expectedCycleTime().toSec());
	ROS_INFO("CM: absolute_path_length_diff=%f", absolute_path_length_diff_);
	ROS_INFO("CM: ratio_path_length_diff=%f", ratio_path_length_diff_);
	ROS_INFO("CM: approach_dist=%f", approach_dist_);
	ROS_INFO("CM: approach_freq=%f", 1/approach_freq_.expectedCycleTime().toSec());

	delay_place_robot_ = ros::Duration(approach_freq_.expectedCycleTime().toSec()/2);

	state_global_ = IDLE;

	srv_make_plan_.request.start.header.frame_id = 	"map";
	srv_make_plan_.request.goal.header.frame_id = 	"map";

	for(int i=0; i<nb_last_vels_; i++)
		last_vels_[i] = geometry_msgs::Twist();
	current_path_.poses.clear();
	previous_path_.poses.clear();

	// Publishers
	pub_log_ = nh_.advertise<std_msgs::String>("log", 100);
	pub_cancel_goal_ =	nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_vel_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("perturbed_cmd", 100);
	pub_perturbed_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("perturbed_cmd", 100);

	// Service servers
	server_check_conflict_ =	nh_.advertiseService("check_conflict", &ConflictManager::srvCheckConflict, this);
	server_init_conflict_ = 		nh_.advertiseService("init_check_conflict", &ConflictManager::srvInitCheckConflict, this);
	server_init_first_path_ = 	nh_.advertiseService("init_first_path_conflict", &ConflictManager::srvInitFirstPath, this);

	// Service clients
	ros::service::waitForService("cancel_goal_and_stop");
	client_cancel_goal_and_stop_ = 	nh_.serviceClient<std_srvs::Empty>("cancel_goal_and_stop");
	ros::service::waitForService("global_planner_path_blockage/make_plan");
	client_make_plan_ =				nh_.serviceClient<navfn::MakeNavPlan>("global_planner_path_blockage/make_plan");
	ros::service::waitForService("update_robot_map");
	client_update_robot_map_ = nh_.serviceClient<std_srvs::Empty>("update_robot_map");
	ros::service::waitForService("resumeSupervisor");
	client_resume_supervisor_ = nh_.serviceClient<std_srvs::Empty>("resumeSupervisor");
}

bool ConflictManager::srvInitCheckConflict(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	//ROS_INFO("CM: Check conflict init");
	for(int i=0; i<nb_last_vels_; i++)
		last_vels_[i] = geometry_msgs::Twist();
	current_path_.poses.clear();
	previous_path_.poses.clear();

	return true;
}

bool ConflictManager::srvInitFirstPath(inhus::ActionBool::Request &req, inhus::ActionBool::Response &res)
{
	srv_make_plan_.request.start.pose.position.x = h_pose_vel_.pose.x;
    srv_make_plan_.request.start.pose.position.y = h_pose_vel_.pose.y;
    srv_make_plan_.request.goal.pose.position.x = req.action.target_pose.pose.position.x;
    srv_make_plan_.request.goal.pose.position.y = req.action.target_pose.pose.position.y;
	
	if(client_make_plan_.call(srv_make_plan_))
	{
		// Get path //
		nav_msgs::Path path;
		path.poses = srv_make_plan_.response.path;

		//ROS_INFO("CM: FIRST !");
		// first w/o robot
		previous_path_ = path;
		std_msgs::String msg;
		current_path_ = path;

		float path_length = computePathLength(&path);
		ros::Time now = ros::Time::now();
		msg.data = "CONFLICT_MANAGER FIRST " + std::to_string(path_length) + " " + std::to_string(now.toSec());
		pub_log_.publish(msg);
	}
	
	return true;
}

bool ConflictManager::srvCheckConflict(inhus::ActionBool::Request &req, inhus::ActionBool::Response &res)
{
	res.conflict = 	false;
	geometry_msgs::PoseStamped goal_pose = req.action.target_pose;

	srv_make_plan_.request.start.pose.position.x = h_pose_vel_.pose.x;
    srv_make_plan_.request.start.pose.position.y = h_pose_vel_.pose.y;
    srv_make_plan_.request.goal.pose.position.x = req.action.target_pose.pose.position.x;
    srv_make_plan_.request.goal.pose.position.y = req.action.target_pose.pose.position.y;
	
	if(client_make_plan_.call(srv_make_plan_))
	{
		if(srv_make_plan_.response.path.size()==0)
			ROS_WARN_NAMED("ConflictManager", "Failed to get path in Conflict Manager");
		// Get path //
		nav_msgs::Path path;
		
		path.poses = srv_make_plan_.response.path;
		float path_length = computePathLength(&path);

		std_msgs::String msg;
		ros::Time now = ros::Time::now();
		msg.data = "CONFLICT_MANAGER PATH " + std::to_string(path_length) + " " + std::to_string(now.toSec());
		pub_log_.publish(msg);
		
		// in order to always have a valid path stored in previous_path_
		// the current_path is always uptaded but the previous is only
		// updated if the current one wasn't empty
		if((int)previous_path_.poses.size()==0)
		{
			//ROS_INFO("CM: FIRST !");
			// first w/o robot
			previous_path_ = path;
			msg.data = "CONFLICT_MANAGER FIRST " + std::to_string(path_length) + " " + std::to_string(now.toSec());
			pub_log_.publish(msg);
		}
		else
		{
			if((int)current_path_.poses.size()!=0)
			{
				// seek pose closest to current_pose from current_path
				// only keep path from current_pose to the end store to previous
				cutPath(current_path_, previous_path_, h_pose_vel_.pose);
			}
		}
		// update current_path
		current_path_ = path;

		// float current_path_length = computePathLength(&current_path_);
		// float previous_path_length = computePathLength(&previous_path_);
		// ROS_INFO("CM: after : current=%f previous=%f", current_path_length, previous_path_length);

		// Check conflict
		if((int)current_path_.poses.size()==0 && (int)previous_path_.poses.size() != 0) // No path
		{
			ROS_INFO("CM: Checked NO_PATH");
			state_blocked_ = NO_PATH;
			res.conflict = 	true;
		}
		else if((int)previous_path_.poses.size() != 0 && (int)current_path_.poses.size()!=0) // Changed too much
		{
			float current_path_length = computePathLength(&current_path_);
			float previous_path_length = computePathLength(&previous_path_);

			if(abs(current_path_length-previous_path_length) > absolute_path_length_diff_ 	// if difference big enough in absolute
			&& current_path_length > ratio_path_length_diff_*previous_path_length)   		// and if difference big enough relatively
			{
				ROS_INFO("CM: Checked PATH_CHANGED_TOO_MUCH %f, %f", current_path_length, previous_path_length);
				state_blocked_ = LONGER;
				res.conflict = 	true;
			}
		}
	}

	// CONFLICT DETECTED
	if(res.conflict)
	{
		// remove robot
		ROS_INFO("CM: remove robot in checked");
		*want_robot_placed_ = false;
		client_update_robot_map_.call(srv_signal_);

		// stop human nav goal
		actionlib_msgs::GoalID goal_id;
		pub_cancel_goal_.publish(goal_id);

		// continue H mv
		mean_vel_.linear.x = 0;
		mean_vel_.linear.y = 0;
		int nb = 0;
		for(int i=0; i<nb_last_vels_; i++)
		{
			if(last_vels_[i].linear.x!=0 || last_vels_[i].linear.y!=0)
			{
				nb++;
				mean_vel_.linear.x += last_vels_[i].linear.x;
				mean_vel_.linear.y += last_vels_[i].linear.y;
				ROS_INFO("CM: linear.x=%f linear.y=%f", last_vels_[i].linear.x, last_vels_[i].linear.y);
			}
		}
		mean_vel_.linear.x /= nb;
		mean_vel_.linear.y /= nb;

		// geometry_msgs::Vector3 vel_global_frame, vel_local_frame;
		// vel_global_frame.x = mean_vel_.linear.x;
		// vel_global_frame.y = mean_vel_.linear.y;
		// vel_global_frame.z = 0;
		// tf2::Quaternion qq;
		// qq.setRPY(0,0,h_pose_vel_.pose.theta);
		// qq.normalize();
		// geometry_msgs::TransformStamped transform;
		// transform.header.frame_id = "map";
		// transform.header.stamp = ros::Time::now();
		// transform.child_frame_id = "map";
		// transform.transform.rotation.x = qq.x();
		// transform.transform.rotation.y = qq.y();
		// transform.transform.rotation.z = qq.z();
		// transform.transform.rotation.w = qq.w();
		// tf2::doTransform(vel_global_frame, vel_local_frame, transform);
		geometry_msgs::Twist mean_local_vel;
		// mean_local_vel.linear.x = vel_local_frame.x;
		// mean_local_vel.linear.y = vel_local_frame.y;
		// mean_vel_ = mean_local_vel;
		// ROS_INFO("mean_vel = (%f,%f)", mean_vel_.linear.x, mean_vel_.linear.y);
		// ROS_INFO("mean_local_vel = (%f,%f)", mean_local_vel.linear.x, mean_local_vel.linear.y);

		mean_local_vel.linear.x = mean_vel_.linear.x*cos(h_pose_vel_.pose.theta) + mean_vel_.linear.y*sin(h_pose_vel_.pose.theta);
		mean_local_vel.linear.y = mean_vel_.linear.y*cos(h_pose_vel_.pose.theta) - mean_vel_.linear.x*sin(h_pose_vel_.pose.theta);

		ROS_INFO("CM: mean after = %f", mean_vel_.linear.x);
		pub_vel_cmd_.publish(mean_local_vel);
		ROS_INFO("CM: ######=> CONFLICT : publish cmd = %f, %f", mean_local_vel.linear.x, mean_local_vel.linear.y);
		
		current_action_ = req.action;
		current_path_.poses.clear();
		state_global_ = APPROACH;
		state_approach_ = FIRST;
		ROS_INFO("CM: switch to APPROACH");

		last_replan_ = ros::Time::now() - approach_freq_.expectedCycleTime();
	}

	return true;
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
			float dist_to_robot = sqrt(pow(h_pose_vel_.pose.x - r_pose_vel_.pose.x,2) + pow(h_pose_vel_.pose.y - r_pose_vel_.pose.y,2));
			if(dist_to_robot <= approach_dist_)
			{
				ROS_INFO("CM: close enough => switch to BLOCKED");

				// switch to BLOCKED
				state_global_ = BLOCKED;

				// place back robot
				*want_robot_placed_ = true;
				client_update_robot_map_.call(srv_signal_);

				// stop human
				client_cancel_goal_and_stop_.call(srv_signal_);

				// last_replan_ = ros::Time::now() - blocked_ask_path_freq_.expectedCycleTime();
				last_replan_ = ros::Time::now() + delay_place_robot_;
				delay_place_robot_.sleep();
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
							ROS_INFO("CM: \t => APPROACH <=");
							ROS_INFO("CM: REPLANNING");
							ROS_INFO("CM: dist=%f", dist_to_robot);

							// if not too close from approach_dist
							if(dist_to_robot > approach_dist_ + replan_dist_stop_)
							{
								move_base_msgs::MoveBaseActionGoal goal;
								goal.goal.target_pose.header.frame_id = "map";
								goal.goal.target_pose.header.stamp = ros::Time::now();
								goal.goal = current_action_;
								pub_goal_move_base_.publish(goal);
							}

							srv_make_plan_.request.start.pose.position.x = 	h_pose_vel_.pose.x;
							srv_make_plan_.request.start.pose.position.y = 	h_pose_vel_.pose.y;
							srv_make_plan_.request.goal.pose.position.x = 	current_action_.target_pose.pose.position.x;
							srv_make_plan_.request.goal.pose.position.y = 	current_action_.target_pose.pose.position.y;
							if(client_make_plan_.call(srv_make_plan_))
							{
								nav_msgs::Path path;
								path.poses = srv_make_plan_.response.path;
								float path_length = computePathLength(&path);
								std_msgs::String msg;
								ros::Time now = ros::Time::now();
								msg.data = "CONFLICT_MANAGER PATH " + std::to_string(path_length) + " " + std::to_string(now.toSec());
								pub_log_.publish(msg);
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
							ROS_INFO("CM: put robot in replanning (for check)");
							state_approach_ = CHECKING;
							last_replan_ = ros::Time::now();
						}
						break;

					// check, with the robot, if the human is still blocked
					case CHECKING:
						if(ros::Time::now() - last_replan_ > approach_freq_.expectedCycleTime())
						{
							ROS_INFO("CM: \t => APPROACH <=");
							ROS_INFO("CM: CHECKING");
							ROS_INFO("CM: dist=%f", dist_to_robot);

							// check if still blocked
							bool still_blocked = true;
							srv_make_plan_.request.start.pose.position.x = 	h_pose_vel_.pose.x;
							srv_make_plan_.request.start.pose.position.y = 	h_pose_vel_.pose.y;
							srv_make_plan_.request.goal.pose.position.x = 	current_action_.target_pose.pose.position.x;
							srv_make_plan_.request.goal.pose.position.y = 	current_action_.target_pose.pose.position.y;
							if(client_make_plan_.call(srv_make_plan_))
							{
								nav_msgs::Path path;
								path.poses = srv_make_plan_.response.path;
								float path_length = computePathLength(&path);
								std_msgs::String msg;
								ros::Time now = ros::Time::now();
								msg.data = "CONFLICT_MANAGER PATH " + std::to_string(path_length) + " " + std::to_string(now.toSec());
								pub_log_.publish(msg);

								//ROS_INFO("CM: make_plan %d", (int)srv_make_plan_.response.plan.poses.size());
								if((int)path.poses.size()!=0) // path found
								{
									float response_path_length = computePathLength(&path);
									float previous_path_length = computePathLength(&previous_path_);
									ROS_INFO("CM: check path length = %f", response_path_length);
									ROS_INFO("CM: check previous path length = %f", previous_path_length);

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
								//ROS_INFO("CM: Still blocked, rm R");
								last_replan_ = ros::Time::now();
								state_approach_ = REMOVE_ROBOT;
							}
							else
							{
								// since the human isn't blocked anymore, switch back to EXEC_PLAN
								ROS_INFO("CM: Not BLOCKED while approaching back to IDLE");
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
							ROS_INFO("CM: remove robot in checking (for replan)");
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

			float qy = r_pose_vel_.pose.y - h_pose_vel_.pose.y;
			float qx = r_pose_vel_.pose.x - h_pose_vel_.pose.x;

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
			if(abs(alpha-h_pose_vel_.pose.theta)>0.1)
			{
				cmd.angular.z=1;
				if(alpha-h_pose_vel_.pose.theta<0)
					cmd.angular.z=-cmd.angular.z;

				if(abs(alpha-h_pose_vel_.pose.theta)>PI)
					cmd.angular.z=-cmd.angular.z;
			}
			pub_perturbed_cmd_.publish(cmd);

			// back to APPROACH if too far
			float dist_to_robot = sqrt(pow(h_pose_vel_.pose.x - r_pose_vel_.pose.x,2) + pow(h_pose_vel_.pose.y - r_pose_vel_.pose.y,2));
			//ROS_INFO("CM: dist=%f", dist_to_robot);
			if(dist_to_robot > approach_dist_*1.2)
			{
				ROS_INFO("CM: Back to APPROACH");

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
				ROS_INFO("CM: \t => BLOCKED <=");
				//ROS_INFO("CM: try to replan");

				srv_make_plan_.request.start.pose.position.x = 	h_pose_vel_.pose.x;
				srv_make_plan_.request.start.pose.position.y = 	h_pose_vel_.pose.y;
				srv_make_plan_.request.goal.pose.position.x = 	current_action_.target_pose.pose.position.x;
				srv_make_plan_.request.goal.pose.position.y = 	current_action_.target_pose.pose.position.y;

				// make plan
				if(client_make_plan_.call(srv_make_plan_))
				{
					nav_msgs::Path path;
					path.poses = srv_make_plan_.response.path;

					float response_path_length2 = computePathLength(&path);
					float previous_path_length2 = computePathLength(&previous_path_);

					ROS_INFO("CM: BLOCK : srv.plan=%f previous=%f", response_path_length2, previous_path_length2);
					// ROS_INFO("BLOCK : srv.plan=%d previous=%d", (int)path.poses.size(), (int)previous_path_.poses.size());
					last_replan_ = ros::Time::now();
					if(!path.poses.empty()) // successfully planned once
					{
						float response_path_length = computePathLength(&path);
						float previous_path_length = computePathLength(&previous_path_);

						ROS_INFO("CM: blocked path length = %f", response_path_length);
						ROS_INFO("CM: blocked previous path length = %f", previous_path_length);

						// If new path is 'good'
						if(previous_path_length == 0								// if no path was found before
						|| abs(response_path_length-previous_path_length)<absolute_path_length_diff_		// if close enough in absolute
						|| response_path_length < ratio_path_length_diff_*previous_path_length)   		// or if close enough relatively
						{
							ROS_INFO("CM: Not BLOCKED back to IDLE");
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

void ConflictManager::updateData(inhus::PoseVel h_pose_vel, inhus::PoseVel r_pose_vel)
{
	h_pose_vel_ = h_pose_vel;
	r_pose_vel_ = r_pose_vel;

	if(h_pose_vel_.vel.linear.x!=0 || h_pose_vel.vel.linear.y!=0)
	{
		for(int i=nb_last_vels_-1; i>0; i--)
			last_vels_[i] = last_vels_[i-1];
		last_vels_[0] = h_pose_vel_.vel;
		// ROS_INFO("CM: last_vels = 1]%f 1]%f 1]%f 1]%f", last_vels_[0].linear.x, last_vels_[1].linear.x, last_vels_[2].linear.x, last_vels_[3].linear.x);
	}
}

///////////////////////////////////////////////////////////

/////////////////////// HUMAN MODEL ///////////////////////

HumanBehaviorModel::HumanBehaviorModel(ros::NodeHandle nh)
: check_see_robot_freq_(1)
, b_random_try_freq_(1)
, b_stop_look_stop_dur_(1)
, b_harass_replan_freq_(1)
, surprise_full_increase_durr_(1)
, surprise_full_decrease_durr_(1)
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
	private_nh.param(std::string("dist_radius_inflation"), dist_radius_inflation_, float(0.5));
	private_nh.param(std::string("surprise_full_increase_durr"), f_nb, float(1.0)); surprise_full_increase_durr_ = ros::Duration(f_nb);
	private_nh.param(std::string("surprise_full_decrease_durr"), f_nb, float(2.0)); surprise_full_decrease_durr_ = ros::Duration(f_nb);
	private_nh.param(std::string("surprise_dist"), surprise_dist_, float(2.0));
	private_nh.param(std::string("b_random_chance_choose"), b_random_chance_choose_, int(30));
	private_nh.param(std::string("b_random_try_freq"), f_nb, float(0.5)); b_random_try_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("b_stop_look_dist_near_robot"), b_stop_look_dist_near_robot_, float(2.0));
	private_nh.param(std::string("b_stop_look_stop_dur"), f_nb, float(2.0)); b_stop_look_stop_dur_ = ros::Duration(f_nb);
	private_nh.param(std::string("b_harass_dist_in_front"), b_harass_dist_in_front_, float(2.0));
	private_nh.param(std::string("b_harass_replan_freq"), f_nb, float(2.0)); b_harass_replan_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("map_name"), map_name_, std::string("laas_adream"));
	ROS_INFO("HBM: => Params HBM:");
	ROS_INFO("HBM: ratio_perturbation_cmd=%f", ratio_perturbation_cmd_);
	ROS_INFO("HBM: fov_int=%d fov=%f", fov_int, fov_);
	ROS_INFO("HBM: check_see_robot_freq=%f", 1/check_see_robot_freq_.expectedCycleTime().toSec());
	ROS_INFO("HBM: delay_forget_robot=%f", delay_forget_robot_.toSec());
	ROS_INFO("HBM: human_radius=%f", human_radius_);
	ROS_INFO("HBM: robot_radius=%f", robot_radius_);
	ROS_INFO("HBM: dist_radius_inflation=%f", dist_radius_inflation_);
	ROS_INFO("HBM: surprise_full_increase_durr=%f", surprise_full_increase_durr_.toSec());
	ROS_INFO("HBM: surprise_full_decrease_durr=%f", surprise_full_decrease_durr_.toSec());
	ROS_INFO("HBM: surprise_dist=%f", surprise_dist_);
	ROS_INFO("HBM: human_radius=%f", human_radius_);
	ROS_INFO("HBM: b_random_chance_choose=%d", b_random_chance_choose_);
	ROS_INFO("HBM: b_random_try_freq=%f", 1/b_random_try_freq_.expectedCycleTime().toSec());
	ROS_INFO("HBM: b_stop_look_dist_near_robot=%f", b_stop_look_dist_near_robot_);
	ROS_INFO("HBM: b_stop_look_stop_dur=%f", b_stop_look_stop_dur_.toSec());
	ROS_INFO("HBM: b_harass_dist_in_front=%f", b_harass_dist_in_front_);
	ROS_INFO("HBM: b_harass_replan_freq=%f", 1/b_harass_replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("HBM: map_name=%s", map_name_.c_str());

	// Subscribers
	sub_h_pose_vel_ = nh_.subscribe("interface/in/human_pose_vel", 100, &HumanBehaviorModel::hPoseVelCallback, this);
	sub_r_pose_vel_ = nh_.subscribe("interface/in/robot_pose_vel", 100, &HumanBehaviorModel::rPoseVelCallback, this);
	sub_cmd_geo_ =		nh_.subscribe("cmd_geo", 100, &HumanBehaviorModel::cmdGeoCallback, this);
	sub_goal_done_ =	nh_.subscribe("goal_done", 100, &HumanBehaviorModel::goalDoneCallback, this);
	sub_set_attitude_ = 	nh_.subscribe("/boss/human/set_attitude", 100, &HumanBehaviorModel::setAttitudeCallback, this);
	sub_new_goal_ =		nh_.subscribe("/boss/human/new_goal", 100, &HumanBehaviorModel::newGoalCallback, this);
	sub_pov_map_ = 		nh_.subscribe("map_pov", 1, &HumanBehaviorModel::povMapCallback, this);

	// Publishers
	pub_h_pose_vel_ = 	nh_.advertise<inhus::PoseVel>("known/human_pose_vel", 100);
	pub_r_pose_vel_ = 	nh_.advertise<inhus::PoseVel>("known/robot_pose_vel", 100);
	pub_new_goal_ = 	nh_.advertise<inhus::Goal>("new_goal", 100);
	pub_perturbed_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("perturbed_cmd", 100);
	pub_goal_move_base_ =	nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_log_ = 		nh_.advertise<std_msgs::String>("log", 100);
	pub_pose_marker_human_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_human_pose", 100);
	pub_pose_marker_robot_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_robot_pose", 100);

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

	//ROS_INFO("HBM: I am human");

	// Init

	current_goal_.type = "pose_goal";

	previous_goal_=current_goal_;

	executing_plan_ = 	false;

	last_check_see_robot_ = ros::Time::now();
	last_seen_robot_ = 	ros::Time::now();
	last_time_ = 		ros::Time::now();
	last_harass_ = 		ros::Time::now();
	time_stopped_=		ros::Time::now();

	see_ = false;
	surprise_last_compute_ = ros::Time::now();
	surprise_seen_ratio_ = 0.0;
	dist_ = 100.0;

	radius_sum_sq_ = human_radius_ + robot_radius_;
	radius_sum_sq_ *= radius_sum_sq_;
	ttc_ = -1.0;

	hcb_ = 		false;
	rcb_ = 		false;

	pmcb_ = false;
	know_robot_pose_ = false;
	want_robot_placed_ = true;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	tf2::Quaternion q;
	q.setRPY(0,0,0);
	marker.pose.orientation.w = q.w();
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();

	// ARROW
	// human
	marker.type = 0;
	marker.id = 0;
	marker.pose.position.z = 0.1;
	marker.scale.x = 1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 1;
	human_pose_marker_.markers.push_back(marker);
	// robot
	marker.color.r = 1;
	marker.color.g = 0;
	marker.color.b = 0;
	robot_pose_marker_.markers.push_back(marker);

	// CYLINDER
	// human
	marker.type = 3;
	marker.id = 1;
	marker.pose.position.z = 0.9;
	marker.scale.x = 0.6;
	marker.scale.y = 0.6;
	marker.scale.z = 1.8;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0;
	marker.color.a = 1;
	human_pose_marker_.markers.push_back(marker);
	// robot
	marker.scale.x = 0.8;
	marker.scale.y = 0.8;
	marker.color.r = 1;
	marker.color.g = 0;
	marker.color.b = 0;
	robot_pose_marker_.markers.push_back(marker);

	// ATTITUDES //
	attitude_ = NONE;
	sub_stop_look_ = WAIT_ROBOT;
	sub_harass_ = INIT;

	// INIT GOALS
	goal_file_name_ = "goals.xml";
	std::string goal_file_path = ros::package::getPath("inhus_navigation") + "/maps/" + map_name_ + "/" + goal_file_name_;
	doc_ = new TiXmlDocument(goal_file_path);
	if(!doc_->LoadFile())
		ROS_ERROR("HBM: Failed to load %s. Error : %s", goal_file_path.c_str(), doc_->ErrorDesc());
	else
		ROS_INFO("HBM: Goals file loaded");
	// Check if file is corresponding with map_name
	TiXmlHandle docHandle(doc_);
	TiXmlElement* l_map = docHandle.FirstChild("map_name").ToElement();
	std::string map_name_read = "";
	if(NULL != l_map->Attribute("name"))
		map_name_read = l_map->Attribute("name");
	if(map_name_read != map_name_)
		ROS_ERROR("HBM: Goals file mismatches the map_name");
	else
		ROS_INFO("HBM: Goals file corresponds with map_name");
	// Extract goals
	this->readGoalsFromXML();
	ROS_INFO("HBM: Goals extracted");
	// Show goals extracted
	// this->showGoals();
}

void HumanBehaviorModel::readGoalsFromXML()
{
	TiXmlHandle docHandle(doc_);
	inhus::Goal goal;

	// Extracting pose_goals
	TiXmlElement* l_goal = docHandle.FirstChild("goals").FirstChild("pose_goals").FirstChild("pose_goal").ToElement();
	while(l_goal)
	{
		if(NULL != l_goal->Attribute("type"))
			goal.type = l_goal->Attribute("type");
		if(goal.type == "pose_goal")
			if(NULL != l_goal->Attribute("x"))
				goal.pose_goal.pose.x = std::stof(l_goal->Attribute("x"));
			if(NULL != l_goal->Attribute("y"))
				goal.pose_goal.pose.y = std::stof(l_goal->Attribute("y"));
			if(NULL != l_goal->Attribute("theta"))
				goal.pose_goal.pose.theta = std::stof(l_goal->Attribute("theta"));
			if(NULL != l_goal->Attribute("radius"))
				goal.pose_goal.radius = std::stof(l_goal->Attribute("radius"));
		known_goals_.push_back(goal);

		l_goal = l_goal->NextSiblingElement("pose_goal");
	}

	// Extracting named_goals
	TiXmlElement* l_named_goal = docHandle.FirstChild("goals").FirstChild("named_goals").FirstChild().ToElement();
	while(l_named_goal)
	{
		// Get type
		if(NULL != l_named_goal->Attribute("type"))
			goal.type = l_named_goal->Attribute("type");
		if(goal.type == "named_goal")
			goal.named_goal.name = l_named_goal->Value();
		known_goals_.push_back(goal);
		l_named_goal = l_named_goal->NextSiblingElement();
	}
}

void HumanBehaviorModel::showGoals()
{
	// list goals
	std::cout << "=> list_goals <=" << std::endl;
	for(unsigned int i=0; i<known_goals_.size(); i++)
	{
		std::cout << "\t" << known_goals_[i].type << " ";
		if(known_goals_[i].type == "pose_goal")
			std::cout << known_goals_[i].pose_goal.pose.x << " " << known_goals_[i].pose_goal.pose.y << " " << known_goals_[i].pose_goal.pose.theta << " " << known_goals_[i].pose_goal.radius << std::endl;
		else if(known_goals_[i].type == "named_goal")
			std::cout << known_goals_[i].named_goal.name << std::endl;
	}
}

void HumanBehaviorModel::publishGoal(inhus::Goal goal)
{
	previous_goal_ = 	current_goal_;
	current_goal_ = 	goal;
	executing_plan_ = true;
	pub_new_goal_.publish(goal);
}

void HumanBehaviorModel::processSimData()
{
	model_h_pose_vel_ = sim_h_pose_vel_;
	model_r_pose_vel_ = sim_r_pose_vel_;
}

void HumanBehaviorModel::publishModelData()
{
	pub_h_pose_vel_.publish(model_h_pose_vel_);
	pub_r_pose_vel_.publish(model_r_pose_vel_);
}

void HumanBehaviorModel::pubDist()
{
	dist_ = sqrt(pow(model_r_pose_vel_.pose.x-model_h_pose_vel_.pose.x,2) + pow(model_r_pose_vel_.pose.y-model_h_pose_vel_.pose.y,2));
	msg_log_.data = "HUMAN_MODEL DIST " + std::to_string(dist_) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg_log_);
}

void HumanBehaviorModel::computeTTC()
{
	ttc_ = -1.0; // ttc infinite

	geometry_msgs::Pose2D C; // robot human relative position
	C.x = model_h_pose_vel_.pose.x - model_r_pose_vel_.pose.x;
	C.y = model_h_pose_vel_.pose.y - model_r_pose_vel_.pose.y;
	double C_sq = C.x*C.x + C.y*C.y; // dot product C.C, distance robot human

	double robot_inflated_radius = robot_radius_*C_sq/dist_radius_inflation_;
	if(robot_inflated_radius < robot_radius_)
		robot_inflated_radius = robot_radius_;
	radius_sum_sq_ = human_radius_ + robot_inflated_radius;
	radius_sum_sq_ *= radius_sum_sq_;
	if(C_sq <= radius_sum_sq_) // already touching
		ttc_ = 0.0;
	else
	{
		geometry_msgs::Twist V; // relative velocity human to robot
		V.linear.x = model_r_pose_vel_.vel.linear.x - model_h_pose_vel_.vel.linear.x;
		V.linear.y = model_r_pose_vel_.vel.linear.y - model_h_pose_vel_.vel.linear.y;

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
		// ROS_INFO("HBM: TTC = %f", ttc_);
		msg_log_.data = "HUMAN_MODEL TTC " + std::to_string(ttc_) + " " + std::to_string(ros::Time::now().toSec());
		pub_log_.publish(msg_log_);
	}
}

void HumanBehaviorModel::computeRelSpd()
{
	relative_speed_ = sqrt(pow(model_h_pose_vel_.vel.linear.x-model_r_pose_vel_.vel.linear.x,2) + pow(model_h_pose_vel_.vel.linear.y-model_r_pose_vel_.vel.linear.y,2));
	msg_log_.data = "HUMAN_MODEL REL_SPD " + std::to_string(relative_speed_) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg_log_);
}

bool HumanBehaviorModel::initDone()
{
	return hcb_ && rcb_ && pmcb_;
}

void HumanBehaviorModel::updatePoseMarkers()
{
	tf2::Quaternion q;
	
	human_pose_marker_.markers[0].pose.position.x = model_h_pose_vel_.pose.x;
	human_pose_marker_.markers[0].pose.position.y = model_h_pose_vel_.pose.y;
	human_pose_marker_.markers[1].pose.position.x = model_h_pose_vel_.pose.x;
	human_pose_marker_.markers[1].pose.position.y = model_h_pose_vel_.pose.y;
	q.setRPY(0,0,model_h_pose_vel_.pose.theta);
	human_pose_marker_.markers[0].pose.orientation.w = q.w();
	human_pose_marker_.markers[0].pose.orientation.x = q.x();
	human_pose_marker_.markers[0].pose.orientation.y = q.y();
	human_pose_marker_.markers[0].pose.orientation.z = q.z();

	robot_pose_marker_.markers[0].pose.position.x = model_r_pose_vel_.pose.x;
	robot_pose_marker_.markers[0].pose.position.y = model_r_pose_vel_.pose.y;
	robot_pose_marker_.markers[1].pose.position.x = model_r_pose_vel_.pose.x;
	robot_pose_marker_.markers[1].pose.position.y = model_r_pose_vel_.pose.y;
	q.setRPY(0,0,model_r_pose_vel_.pose.theta);
	robot_pose_marker_.markers[0].pose.orientation.w = q.w();
	robot_pose_marker_.markers[0].pose.orientation.x = q.x();
	robot_pose_marker_.markers[0].pose.orientation.y = q.y();
	robot_pose_marker_.markers[0].pose.orientation.z = q.z();

	pub_pose_marker_human_.publish(human_pose_marker_);
	pub_pose_marker_robot_.publish(robot_pose_marker_);
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
	if(A_map_x < 0 || A_map_x >= (int)g_map_[0].size() || A_map_y < 0 || A_map_y >= (int)g_map_.size()
	|| B_map_x < 0 || B_map_x >= (int)g_map_[0].size() || B_map_y < 0 || B_map_y >= (int)g_map_.size())
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
				//ROS_INFO("HBM: place_robot true");
				know_robot_pose_ = true;
				srv_place_robot_.request.data = true;
				client_place_robot_.call(srv_place_robot_);
			}
		}
		else
		{
			if(know_robot_pose_) // falling edge
			{
				//ROS_INFO("HBM: place_robot false");
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
					//ROS_INFO("HBM: place_robot false");
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
				//ROS_INFO("HBM: place_robot false");
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
		geometry_msgs::Pose2D human_pose_offset = model_h_pose_vel_.pose;
		human_pose_offset.x -= offset_pov_map_x_;
		human_pose_offset.y -= offset_pov_map_y_;
		geometry_msgs::Pose2D robot_pose_offset = model_r_pose_vel_.pose;
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
				//ROS_INFO("HBM: I SEE");
				see_ = true;
				last_seen_robot_ = ros::Time::now();
			}
			else
			{
				// human can't see the robot
				//ROS_INFO("HBM: VIEW IS BLOCKED");
				see_ = false;
			}
		}
		else
		{
			//ROS_INFO("HBM: NOT IN FOV");
			see_ = false;
		}

		// Update robot on map if needed
		this->updateRobotOnMap();

		last_check_see_robot_ = ros::Time::now();
	}
}

void HumanBehaviorModel::computeSurprise()
{
	ros::Duration delta_t = ros::Time::now() - surprise_last_compute_;
	// std::cout << "delta_t = " << delta_t.toSec() << " ";

	if(see_)
	{
		// std::cout << "seen ";

		// Compute the seen ratio
		surprise_seen_ratio_ += delta_t.toSec()/surprise_full_increase_durr_.toSec();
		if(surprise_seen_ratio_ > 1.0)
			surprise_seen_ratio_ = 1.0;

		// Test if seen ratio high enough compared to distance
		// std::cout << "dist=" << dist_ << " "; 
		if(dist_ < surprise_dist_ && surprise_seen_ratio_ < 0.6)
		{
			// Human is surprised
			// std::cout << "SURPRISED ";
			msg_log_.data = "HUMAN_MODEL SURPRISED " + std::to_string(ros::Time::now().toSec());
			pub_log_.publish(msg_log_);
		}
	}
	else
	{
		// std::cout << "not_seen ";

		// slowly decrease the seen_ratio
		surprise_seen_ratio_ -= delta_t.toSec()/surprise_full_decrease_durr_.toSec();
		if(surprise_seen_ratio_ < 0.0)
			surprise_seen_ratio_ = 0.0;
	}

	// std::cout << "ratio=" << surprise_seen_ratio_ << std::endl;

	msg_log_.data = "HUMAN_MODEL SEEN_RATIO " + std::to_string(surprise_seen_ratio_) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg_log_);

	surprise_last_compute_ = ros::Time::now();
}

////////////////////// Attitudes //////////////////////////

inhus::Goal HumanBehaviorModel::chooseGoal(bool random)
{
	inhus::Goal goal;
	static int index_list=-1;
	int i=0;

	if(random)
	{
		if(previous_goal_.type == "pose_goal")
		{
			do
			{
				i=rand()%known_goals_.size();
			}while(known_goals_[i].pose_goal.pose.x==previous_goal_.pose_goal.pose.x && known_goals_[i].pose_goal.pose.y==previous_goal_.pose_goal.pose.y);
		}
		else if(previous_goal_.type == "named_goal")
		{
			do
			{
				i=rand()%known_goals_.size();
			}while(known_goals_[i].named_goal.name == previous_goal_.named_goal.name);
		}
	}
	// follow list of known goals
	else
	{
		index_list=(index_list+1)%known_goals_.size();
		i = index_list;
	}

	current_goal_=goal;

	return goal;
}

void HumanBehaviorModel::attNonStop()
{
	if(!executing_plan_)
	{
		inhus::Goal goal = chooseGoal(true);

		this->publishGoal(goal);
	}
}

void HumanBehaviorModel::attRandom()
{
	if(ros::Time::now()-last_time_> b_random_try_freq_.expectedCycleTime())
	{
		int nb = rand()%100 + 1;
		//ROS_INFO("HBM: Tirage %d/%d", nb, b_random_chance_choose_);
		if(nb < b_random_chance_choose_)
		{
			//ROS_INFO("HBM: DECIDE NEW GOAL ! ");
			inhus::Goal new_goal = this->chooseGoal(true);
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
				float dist = sqrt(pow(model_h_pose_vel_.pose.x-model_r_pose_vel_.pose.x,2) + pow(model_h_pose_vel_.pose.y-model_r_pose_vel_.pose.y,2));
				//ROS_INFO("HBM: threshold=%f dist=%f", b_stop_look_dist_near_robot_, dist);
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
					float qy = model_r_pose_vel_.pose.y - model_h_pose_vel_.pose.y;
					float qx = model_r_pose_vel_.pose.x - model_h_pose_vel_.pose.x;

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
					if(abs(alpha-model_h_pose_vel_.pose.theta)>0.1)
					{
						cmd.angular.z=2;
						if(alpha-model_h_pose_vel_.pose.theta<0)
							cmd.angular.z=-cmd.angular.z;

						if(abs(alpha-model_h_pose_vel_.pose.theta)>PI)
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
				float dist = sqrt(pow(model_h_pose_vel_.pose.x-model_r_pose_vel_.pose.x,2) + pow(model_h_pose_vel_.pose.y-model_r_pose_vel_.pose.y,2));
				if(dist>b_stop_look_dist_near_robot_)
				{
					//ROS_INFO("HBM: Reset STOP_LOOK");
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
				in_front.x = model_r_pose_vel_.pose.x + cos(model_r_pose_vel_.pose.theta)*b_harass_dist_in_front_;
				in_front.y = model_r_pose_vel_.pose.y + sin(model_r_pose_vel_.pose.theta)*b_harass_dist_in_front_;
				in_front.theta = model_r_pose_vel_.pose.theta-3.14;

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
	conflict_manager_->updateData(model_h_pose_vel_, model_r_pose_vel_);
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

void HumanBehaviorModel::hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	sim_h_pose_vel_.pose.x=msg->pose.x;
	sim_h_pose_vel_.pose.y=msg->pose.y;
	sim_h_pose_vel_.pose.theta=msg->pose.theta;
	sim_h_pose_vel_.vel=msg->vel;

	if(!hcb_)
	{
		//ROS_INFO("HBM: hcb");
		hcb_=true;
	}
}

void HumanBehaviorModel::rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	sim_r_pose_vel_.pose.x=msg->pose.x;
	sim_r_pose_vel_.pose.y=msg->pose.y;
	sim_r_pose_vel_.pose.theta=msg->pose.theta;
	sim_r_pose_vel_.vel=msg->vel;

	if(!rcb_)
	{
		//ROS_INFO("HBM: rcb");
		rcb_=true;
	}
}

void HumanBehaviorModel::cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// if(msg->linear.x!=0 || msg->linear.y!=0)
	if(!(conflict_manager_->inApproach() && msg->linear.x==0 && msg->linear.y==0))
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
}

void HumanBehaviorModel::goalDoneCallback(const inhus::Goal::ConstPtr& msg)
{
	//ROS_INFO("HBM: goal done !!!!!!!");
	pub_perturbed_cmd_.publish(geometry_msgs::Twist());
	executing_plan_ = false;
}

void HumanBehaviorModel::newGoalCallback(const inhus::Goal::ConstPtr& goal)
{
	this->publishGoal(*goal);
}

void HumanBehaviorModel::setAttitudeCallback(const std_msgs::Int32::ConstPtr& msg)
{
	bool changed=false;
	switch(msg->data)
	{
		case NONE: //0
			ROS_INFO("HBM: Attitude set : NONE");
			changed=true;
			break;
		case NON_STOP: //1
			ROS_INFO("HBM: Attitude set : NON_STOP");
			changed=true;
			break;
		case RANDOM: //2
			ROS_INFO("HBM: Attitude set : RANDOM");
			changed=true;
			break;
		case STOP_LOOK: //3
			ROS_INFO("HBM: Attitude set : STOP_LOOK");
			changed=true;
			break;
		case HARASS: //4
			ROS_INFO("HBM: Attitude set : HARASS");
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
		//ROS_INFO("HBM: pmcb");
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

		// Update visualization markers of human and robot
		human_model.updatePoseMarkers();

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

		// Compute surprise effect
		human_model.computeSurprise();

		ros::spinOnce();
		rate.sleep();
	}
}

///////////////////////////////////////////////////////////

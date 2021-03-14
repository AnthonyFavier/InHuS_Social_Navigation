#include "supervisor.h"

///////////////////////////// SUPERVISOR /////////////////////////////////

Supervisor::Supervisor()
: plan_()
, replan_freq_(1) // init Rates and durations
, place_robot_delay_(1)
{
	// Ros Params
	float f_nb;
	nh_.param(std::string("replan_freq"), f_nb, float(2.0)); replan_freq_ = ros::Rate(f_nb);
	nh_.param(std::string("replan_dist_stop"), replan_dist_stop_, float(0.5));
	nh_.param(std::string("place_robot_delay"), f_nb, float(0.3)); place_robot_delay_ = ros::Duration(f_nb);
	ROS_INFO("Params:");
	ROS_INFO("replan_freq=%f", 1/replan_freq_.expectedCycleTime().toSec());
	ROS_INFO("replan_dist_stop=%f", replan_dist_stop_);
	ROS_INFO("place_robot_delay=%f", place_robot_delay_.toSec());

	// Service clients
	client_plan_ = 					nh_.serviceClient<human_sim::ComputePlan>("compute_plan");
	client_place_robot_hm_ =		nh_.serviceClient<move_human::PlaceRobot>("place_robot_hm");
	client_check_conflict_ =		nh_.serviceClient<human_sim::ActionBool>("check_conflict");
	client_init_check_conflict_ = 	nh_.serviceClient<human_sim::Signal>("init_check_conflict");

	// Service servers
	server_set_wait_goal_ =		nh_.advertiseService("set_wait_goal", &Supervisor::srvSetWaitGoal, this);
	server_suspend_ =			nh_.advertiseService("suspend", &Supervisor::srvSuspend, this);
	server_back_exec_plan_ =	nh_.advertiseService("back_exec_plan", &Supervisor::srvBackExecPlan, this);

	// Subscribers
	sub_human_pose_ = 		nh_.subscribe("known/human_pose", 100, &Supervisor::humanPoseCallback, this);
	sub_robot_pose_ = 		nh_.subscribe("known/robot_pose", 100, &Supervisor::robotPoseCallback, this);
	sub_new_goal_  = 		nh_.subscribe("new_goal", 100, &Supervisor::newGoalCallback, this);
	sub_path_ =				nh_.subscribe("move_base/GlobalPlanner/plan", 100, &Supervisor::pathCallback, this);
	sub_status_move_base_ = nh_.subscribe("move_base/status", 100, &Supervisor::stateMoveBaseCB, this);

	// Publishers
	pub_goal_move_base_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_goal_done_ = 	nh_.advertise<human_sim::Goal>("goal_done", 100);
	pub_log_ =			nh_.advertise<std_msgs::String>("log", 100);
	pub_marker_rviz_ =	nh_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	pub_stop_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("stop_cmd", 100);

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

	global_state_ = WAIT_GOAL;
	goal_received_ = false;
	goal_status_.status = 0;

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	robot_pose_.x = 0;
	robot_pose_.y = 0;
	robot_pose_.theta = 0;

	ros::service::waitForService("compute_plan");
	//ROS_INFO("Connected to taskPlanner server");
}

void Supervisor::FSM()
{
	switch(global_state_)
	{
		case WAIT_GOAL:
		// Wait for goal from HumanBehaviorModel
			ROS_INFO("\t => WAIT_GOAL <=");
			if(goal_received_)
			{
				goal_received_ = false;
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
					global_state_ = WAIT_GOAL;
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
							/*else if(goal_status_.status == 2) // PREEMPTED
							{
								ROS_INFO("PREEMPTED");
								this->updateMarkerPose(0, 0, 0);
								global_state_ = WAIT_GOAL;

							}*/
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
							break;
					}

					plan_.updateState();
				}
			}
			break;

		case SUSPENDED:
			ROS_INFO("\t => SUSPENDED <=");
			break;
	}
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

	current_goal_.type=msg->type;
	current_goal_.x=msg->x;
	current_goal_.y=msg->y;
	current_goal_.theta=msg->theta;
}

bool Supervisor::srvSetWaitGoal(human_sim::Signal::Request &req, human_sim::Signal::Response &res)
{
	ROS_INFO("WAIT_GOAL SET !!!");
	global_state_ = WAIT_GOAL;

	return true;
}

bool Supervisor::srvSuspend(human_sim::Signal::Request &req, human_sim::Signal::Response &res)
{
	global_state_=SUSPENDED;
	return true;
}

bool Supervisor::srvBackExecPlan(human_sim::Signal::Request& req, human_sim::Signal::Response& res)
{
	global_state_=EXEC_PLAN;
	last_replan_ = ros::Time::now() - replan_freq_.expectedCycleTime();
	return true;
}

void Supervisor::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
	current_path_ = *path;
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

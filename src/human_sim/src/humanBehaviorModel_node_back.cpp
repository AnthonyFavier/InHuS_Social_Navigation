#include "humanBehaviorModel.h"

/////////////////////// HUMAN MODEL ///////////////////////

HumanBehaviorModel::HumanBehaviorModel()
: b_stop_look_stop_dur_(1)
, b_harass_replan_freq_(1)
, b_random_try_freq_(1)
, check_see_robot_freq_(1)
{
	srand(time(NULL));

	// Ros Params
	ros::NodeHandle private_nh("~");
	std::string str; float f_nb; int fov_int;
	private_nh.param(std::string("human_radius"), human_radius_, float(0.25));
	private_nh.param(std::string("robot_radius"), robot_radius_, float(0.3));
	private_nh.param(std::string("ratio_perturbation_cmd"), ratio_perturbation_cmd_, float(0.0));
	private_nh.param(std::string("b_random_chance_choose"), b_random_chance_choose_, int(30));
	private_nh.param(std::string("b_random_try_freq"), f_nb, float(0.5)); b_random_try_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("b_stop_look_dist_near_robot"), b_stop_look_dist_near_robot_, float(2.0));
	private_nh.param(std::string("b_stop_look_stop_dur"), f_nb, float(2.0)); b_stop_look_stop_dur_ = ros::Duration(f_nb);
	private_nh.param(std::string("b_harass_dist_in_front"), b_harass_dist_in_front_, float(2.0));
	private_nh.param(std::string("b_harass_replan_freq"), f_nb, float(2.0)); b_harass_replan_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("fov"), fov_int, int(180)); fov_ = fov_int*PI/180;
	private_nh.param(std::string("check_see_robot_freq"), f_nb, float(2.0)); check_see_robot_freq_ = ros::Rate(f_nb);
	private_nh.param(std::string("delay_forget_robot"), f_nb, float(1.5)); delay_forget_robot_ = ros::Duration(f_nb);

	//ROS_INFO("Params:");
	//ROS_INFO("human_radius=%f", human_radius_);
	//ROS_INFO("robot_radius=%f", robot_radius_);
	//ROS_INFO("ratio_perturbation_cmd=%f", ratio_perturbation_cmd_);
	//ROS_INFO("b_random_try_freq=%f", b_random_try_freq_.expectedCycleTime().toSec());
	//ROS_INFO("b_random_chance_choose=%d", b_random_chance_choose_);
	//ROS_INFO("b_stop_look_dist_near_robot=%f", b_stop_look_dist_near_robot_);
	//ROS_INFO("b_stop_look_stop_dur=%f", b_stop_look_stop_dur_.toSec());
	//ROS_INFO("b_harass_dist_in_front=%f", b_harass_dist_in_front_);
	//ROS_INFO("b_harass_replan_freq=%f", b_harass_replan_freq_.expectedCycleTime().toSec());
	//ROS_INFO("fov_int=%d fov=%f", fov_int, fov_);
	//ROS_INFO("check_see_robot_freq=%f", check_see_robot_freq_.expectedCycleTime().toSec());
	//ROS_INFO("delay_forget_robot=%f", delay_forget_robot_.toSec());

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
	sub_pov_map_ = 		nh_.subscribe("pov_map", 1, &HumanBehaviorModel::povMapCallback, this);

	// Publishers
	pub_human_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("known/human_pose", 100);
	pub_human_vel_ = 	nh_.advertise<geometry_msgs::Twist>("known/human_vel", 100);
	pub_robot_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("known/robot_pose", 100);
	pub_robot_vel_ = 	nh_.advertise<geometry_msgs::Twist>("known/robot_vel", 100);
	pub_new_goal_ = 	nh_.advertise<human_sim::Goal>("new_goal", 100);
	pub_perturbed_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("perturbed_cmd", 100);
	pub_goal_move_base_ =	nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_log_ = 		nh_.advertise<std_msgs::String>("log", 100);

	// Service clients
	client_set_get_goal_ = 		nh_.serviceClient<human_sim::Signal>("set_get_goal");
	client_cancel_goal_and_stop_ = 	nh_.serviceClient<human_sim::Signal>("cancel_goal_and_stop");
	client_place_robot_ = 		nh_.serviceClient<move_human::PlaceRobot>("place_robot");

	// Service server
	server_place_robot_ = nh_.advertiseService("place_robot_hm", &HumanBehaviorModel::srvPlaceRobotHM, this);

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

	current_goal_.type = 	"Position";
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

	radius_sum_sq_ = human_radius_ + robot_radius_;
	radius_sum_sq_ *= radius_sum_sq_;
	ttc_ = -1.0;

	hcb_ = 		false;
	rcb_ = 		false;

	pmcb_ = false;
	know_robot_pose_ = false;
	supervisor_wants_robot_ = true;

	// ATTITUDES //
	attitude_ = NONE;
	sub_stop_look_ = WAIT_ROBOT;
	sub_harass_ = INIT;

	// INIT GOALS //
	GoalArea area;
	area.goal.type="Position";

	//1//
	area.goal.x=1.0; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	known_goals_.push_back(area);
	//2//
	area.goal.x=3.15; 	area.goal.y=3.2; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	//3//
	area.goal.x=10.2; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	known_goals_.push_back(area);
	//4//
	area.goal.x=7.90; 	area.goal.y=5.1; 	area.goal.theta=-PI/2;	area.radius=0;
	known_goals_.push_back(area);
	//5//
	area.goal.x=7.8; 	area.goal.y=9.98; 	area.goal.theta=-PI;	area.radius=0;
	known_goals_.push_back(area);
	//6//
	area.goal.x=3.42; 	area.goal.y=9.48; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	//7//
	area.goal.x=4.72; 	area.goal.y=17.68; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	//8//
	area.goal.x=10.6; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	known_goals_.push_back(area);
	//9//
	area.goal.x=1.0; 	area.goal.y=15.8; 	area.goal.theta=-PI;	area.radius=0;
	known_goals_.push_back(area);
	//10//
	area.goal.x=0.65; 	area.goal.y=8.50; 	area.goal.theta=-PI;	area.radius=0;
	known_goals_.push_back(area);

	//11//
	area.goal.x=8.8; 	area.goal.y=0.8; 	area.goal.theta=0;	area.radius=1.3;
	known_goals_.push_back(area);
	//12//
	area.goal.x=3.0; 	area.goal.y=15.3; 	area.goal.theta=0;	area.radius=2;
	known_goals_.push_back(area);
	//13//
	area.goal.x=8.0; 	area.goal.y=15.5; 	area.goal.theta=0;	area.radius=2;
	known_goals_.push_back(area);
}

void HumanBehaviorModel::publishGoal(human_sim::Goal& goal)
{
	executing_plan_ = true;
	pub_new_goal_.publish(goal);
	//ROS_INFO("goal published");
}

human_sim::Goal HumanBehaviorModel::chooseGoal(bool random)
{
	human_sim::Goal goal;
	static int index_list=-1;
	int i=0;

	if(random)
	{
		do
		{
			i=rand()%10 +1 ;
		}while(known_goals_[i].goal.x==previous_goal_.x && known_goals_[i].goal.y==previous_goal_.y);
	}
	// follow list of known goals
	else
	{
		index_list=(index_list+1)%known_goals_.size();
		i = index_list;
	}

	//ROS_INFO("i=%d", i);
	//ROS_INFO("x=%f, y=%f, theta=%f, radius=%f", known_goals_[i].goal.x,known_goals_[i].goal.y,known_goals_[i].goal.theta,known_goals_[i].radius);

	// if it's an area, pick a goal in it
	if(known_goals_[i].radius!=0)
	{
		float alpha = (rand()%(2*314))/100 - PI;
		float r = (rand()%(int(known_goals_[i].radius*100)))/100.0;

		//ROS_INFO("alpha=%f, r=%f", alpha, r);

		goal.type = "Position";
		goal.x = known_goals_[i].goal.x + r * cos(alpha);
		goal.y = known_goals_[i].goal.y + r * sin(alpha);
		goal.theta = 0;
	}
	else
		goal = known_goals_[i].goal;

	current_goal_=goal;

	//ROS_INFO("goal choosen !");
	//ROS_INFO("%s (%f, %f, %f)", goal.type.c_str(), goal.x, goal.y, goal.theta);

	return goal;
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

void HumanBehaviorModel::nonStop()
{
//	ROS_INFO("NON_STOP");
	if(!executing_plan_)
	{
		//ROS_INFO("CHOOSED new goal in NON STOP");
		human_sim::Goal goal = chooseGoal(true);

		this->publishGoal(goal);
	}
}

void HumanBehaviorModel::newRandomGoalGeneration()
{
	if(ros::Time::now()-last_time_> b_random_try_freq_.expectedCycleTime())
	{
		int nb = rand()%100 + 1;
		//ROS_INFO("Tirage %d/%d", nb, b_random_chance_choose_);
		if(nb < b_random_chance_choose_)
		{
			//ROS_INFO("DECIDE NEW GOAL ! ");
			human_sim::Goal previous_goal = current_goal_;
			human_sim::Goal new_goal = this->chooseGoal(true);
			if(new_goal.x != previous_goal.x || new_goal.y != previous_goal.y)
			{
				this->publishGoal(new_goal);
				//ROS_INFO("published");
			}
			else
			{
				//ROS_INFO("ALREADY GOING!");
			}
		}
		last_time_=ros::Time::now();
	}
}

void HumanBehaviorModel::stopLookRobot()
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
				//ROS_INFO("current_goal = %f,%f", current_goal_.x, current_goal_.y);
				//ROS_INFO("previous_goal = %f,%f", previous_goal_.x, previous_goal_.y);

				// Stop goal and motion
				//ROS_INFO("Stopped !");
				human_sim::Signal srv_cancel;
				client_cancel_goal_and_stop_.call(srv_cancel);

				// Set global FSM to WAIT_GOAL
				human_sim::Signal srv_set;
				client_set_get_goal_.call(srv_set);

				// Get time
				time_stopped_ = ros::Time::now();

				sub_stop_look_=LOOK_AT_ROBOT;
				break;
			}

		case LOOK_AT_ROBOT:
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

				if(ros::Time::now() - time_stopped_ > b_stop_look_stop_dur_)
					sub_stop_look_=RESUME_GOAL;
				break;
			}

		case RESUME_GOAL:
			//ROS_INFO("Resume Goal");
			if(executing_plan_)
			{
				// resume current goal
				pub_new_goal_.publish(current_goal_);
				//ROS_INFO("sent");
				sub_stop_look_=OVER;
			}
			else
			{
				//ROS_INFO("No previous goal");
				sub_stop_look_=OVER;
			}
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

void HumanBehaviorModel::harassRobot()
{
	switch(sub_harass_)
	{
		case INIT:
			{
				human_sim::Signal srv_cancel;
				client_cancel_goal_and_stop_.call(srv_cancel);

				human_sim::Signal srv_set;
				client_set_get_goal_.call(srv_set);
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
			this->nonStop();
			break;

		case RANDOM:
			this->newRandomGoalGeneration();
			break;

		case STOP_LOOK:
			this->stopLookRobot();
			break;

		case HARASS:
			this->harassRobot();
			break;

		default:
			attitude_=NONE;
			break;
	}
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

bool HumanBehaviorModel::initDone()
{
	return hcb_ && rcb_ && pmcb_;
}

bool HumanBehaviorModel::srvPlaceRobotHM(move_human::PlaceRobot::Request& req, move_human::PlaceRobot::Response& res)
{
	supervisor_wants_robot_ = req.data;

	return true;
}

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
		bool see;
		if(this->testFOV(robot_pose_offset, human_pose_offset, fov_))
		{
			// check if there are obstacles blocking the human view of the robot
			if(this->testObstacleView(human_pose_offset, robot_pose_offset))
			{
				// the human sees the robot
				//ROS_INFO("I SEE");
				see = true;
				last_seen_robot_ = ros::Time::now();
			}
			else
			{
				// human can't see the robot
				//ROS_INFO("VIEW IS BLOCKED");
				see = false;
			}
		}
		else
		{
			//ROS_INFO("NOT IN FOV");
			see = false;
		}

		// Update robot on map if needed
		if(see)
		{
			if(supervisor_wants_robot_)
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
			if(supervisor_wants_robot_)
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

		last_check_see_robot_ = ros::Time::now();
	}
}

////////////////////// Callbacks//////////////////////////

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

void HumanBehaviorModel::goalDoneCallback(const human_sim::Goal::ConstPtr& msg)
{
	//ROS_INFO("goal done !!!!!!!");
	previous_goal_ = current_goal_;
	executing_plan_ = false;
}

void HumanBehaviorModel::newGoalCallback(const human_sim::Goal::ConstPtr& goal)
{
	previous_goal_.x = 	current_goal_.x;
	previous_goal_.y = 	current_goal_.y;
	previous_goal_.theta = 	current_goal_.theta;

	current_goal_.x = 	goal->x;
	current_goal_.y = 	goal->y;
	current_goal_.theta = 	goal->theta;

	//ROS_INFO("CB current_goal = %.2f,%.2f", current_goal_.x, current_goal_.y);
	//ROS_INFO("CB previous_goal = %.2f,%.2f", previous_goal_.x, previous_goal_.y);

	this->publishGoal(current_goal_);
}

void HumanBehaviorModel::setAttitudeCallback(const std_msgs::Int32::ConstPtr& msg)
{
	bool changed=false;
	switch(msg->data)
	{
		case NONE: //0
			//ROS_INFO("Attitude set : NONE");
			changed=true;
			break;
		case NON_STOP: //1
			//ROS_INFO("Attitude set : NON_STOP");
			changed=true;
			break;
		case RANDOM: //2
			//ROS_INFO("Attitude set : RANDOM");
			changed=true;
			break;
		case STOP_LOOK: //3
			//ROS_INFO("Attitude set : STOP_LOOK");
			changed=true;
			break;
		case HARASS: //4
			//ROS_INFO("Attitude set : HARASS");
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	HumanBehaviorModel human_model;

	ros::Rate rate(15);

	//ROS_INFO("Waiting for init ...");
	while(ros::ok() && !human_model.initDone())
	{
		ros::spinOnce();
		rate.sleep();
	}
	//ROS_INFO("LETS_GO");

	while(ros::ok())
	{
		// Process data from simu
		human_model.processSimData();

		// Update robot pose knowledge
		human_model.testSeeRobot();

		// Compute TTC
		human_model.computeTTC();

		// Publish data as perceived by the human model
		human_model.publishModelData();

		// Add perturbation in human attitudes
		human_model.attitudes();

		// Publish human/robot distance to log
		human_model.pubDist();

		rate.sleep();
		ros::spinOnce();
	}
}

///////////////////////////////////////////////////////////

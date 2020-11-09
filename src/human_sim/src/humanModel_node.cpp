#include "humanModel.h"

bool hcb=false;
bool rcb=false;

/////////////////////// HUMAN MODEL ///////////////////////

HumanModel::HumanModel()
: ratio_perturbation_(0)  //=0.2; => +/- 20% of value
, chance_decide_new_goal_(30) //x%
, delay_think_about_new_goal_(ros::Duration(2)) // x% chance every 2s
, dist_near_robot_(2)
, dist_in_front_(2)
, delay_harass_replan_(ros::Duration(0.5))
{
	srand(time(NULL));

	sub_pose_ = 	 	nh_.subscribe("sim/human_pose", 100, &HumanModel::poseCallback, this);
	sub_robot_pose_ =	nh_.subscribe("sim/robot_pose", 100, &HumanModel::robotPoseCallback, this);
	sub_cmd_geo_ =		nh_.subscribe("cmd_geo", 100, &HumanModel::cmdGeoCallback, this);
	sub_goal_done_ =	nh_.subscribe("goal_done", 100, &HumanModel::goalDoneCallback, this);
	sub_set_behavior_ = 	nh_.subscribe("/boss/human/set_behavior", 100, &HumanModel::setBehaviorCallback, this);
	sub_new_goal_ =		nh_.subscribe("/boss/human/new_goal", 100, &HumanModel::newGoalCallback, this);

	pub_new_goal_ = 	nh_.advertise<human_sim::Goal>("/boss/human/new_goal", 100);
	pub_human_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/human_pose", 100);
	pub_robot_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/robot_pose", 100);
	pub_perturbated_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("controller/perturbated_cmd", 100);
	pub_op_mode_ = 		nh_.advertise<std_msgs::Int32>("/boss/human/operating_mode", 100);
	pub_goal_move_base_ =	nh_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
	pub_log_ = 		nh_.advertise<std_msgs::String>("log", 100);

	service_ = 		nh_.advertiseService("choose_goal", &HumanModel::chooseGoalSrv, this);

	client_set_get_goal_ = 		nh_.serviceClient<human_sim::SetGetGoal>("set_get_goal");
	client_cancel_goal_and_stop_ = 	nh_.serviceClient<human_sim::CancelGoalAndStop>("cancel_goal_and_stop");
	client_get_choice_goal_decision_ = nh_.serviceClient<human_sim::GetChoiceGoalDecision>("get_choiceGoalDecision");

	printf("I am human\n");

	Pose2D zero;
	zero.x = 	0;
	zero.y = 	0;
	zero.theta = 	0;
	sim_pose_ = 	   zero;
	sim_robot_pose_=    zero;
	model_pose_ = 	   zero;
	model_robot_pose_ = zero;

	current_goal_.type = 	"Position";
	current_goal_.x  =	0;
	current_goal_.y = 	0;
	current_goal_.theta = 	0;

	previous_goal_=current_goal_;

	was_in_autonomous_ = false;

	last_time_ = ros::Time::now();
	last_harass_ = ros::Time::now();

	// BEHAVIORS //
	behavior_ = NONE;
	sub_stop_look_ = WAIT_ROBOT;
	sub_harass_ = INIT;

	// INIT GOALS //
	human_sim::Goal goal;
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
	area.goal.x=1.15; 	area.goal.y=6.52; 	area.goal.theta=-PI;	area.radius=0;
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

void HumanModel::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	hcb=true;

	sim_pose_.x=msg->x;
	sim_pose_.y=msg->y;
	sim_pose_.theta=msg->theta;
}

void HumanModel::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	rcb=true;

	sim_robot_pose_.x=msg->x;
	sim_robot_pose_.y=msg->y;
	sim_robot_pose_.theta=msg->theta;
}

void HumanModel::cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist perturbated_cmd;

	perturbated_cmd.linear.x = msg->linear.x * (1 + ratio_perturbation_*((float)(rand()%300-100)/100.0));
	perturbated_cmd.linear.y = msg->linear.y * (1 + ratio_perturbation_*((float)(rand()%300-100)/100.0));
	perturbated_cmd.linear.z = 0;
	perturbated_cmd.angular.x = 0;
	perturbated_cmd.angular.y = 0;
	perturbated_cmd.angular.z = msg->angular.z; // no angular perturbation

	pub_perturbated_cmd_.publish(perturbated_cmd);
}

void HumanModel::goalDoneCallback(const human_sim::Goal::ConstPtr& msg)
{
	printf("received !!\n");
	previous_goal_ = current_goal_;
}

bool HumanModel::chooseGoalSrv(human_sim::ChooseGoal::Request& req, human_sim::ChooseGoal::Response& res)
{
	human_sim::Goal goal = this->chooseGoal(false);

	res.goal.type = 	goal.type;
	res.goal.x = 		goal.x;
	res.goal.y = 		goal.y;
	res.goal.theta = 	goal.theta;

}

human_sim::Goal HumanModel::chooseGoal(bool random)
{
	human_sim::Goal goal;
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

	printf("i=%d\n", i);
	printf("x=%f, y=%f, theta=%f, radius=%f\n", known_goals_[i].goal.x,known_goals_[i].goal.y,known_goals_[i].goal.theta,known_goals_[i].radius);

	// if it's an area, pick a goal in it
	if(known_goals_[i].radius!=0)
	{
		float alpha = (rand()%(2*314))/100 - PI;
		float r = (rand()%(int(known_goals_[i].radius*100)))/100.0;

		printf("alpha=%f, r=%f\n", alpha, r);

		goal.type = "Position";
		goal.x = known_goals_[i].goal.x + r * cos(alpha);
		goal.y = known_goals_[i].goal.y + r * sin(alpha);
		goal.theta = 0;
	}
	else
		goal = known_goals_[i].goal;

	current_goal_=goal;

	printf("goal choosen !\n");
	printf("%s (%f, %f, %f)\n", goal.type.c_str(), goal.x, goal.y, goal.theta);

	return goal;
}

void HumanModel::processSimData()
{
	model_pose_ = sim_pose_;
	model_robot_pose_ = sim_robot_pose_;
}

void HumanModel::publishModelData()
{
	geometry_msgs::Pose2D pose;
	pose.x = model_pose_.x;
	pose.y = model_pose_.y;
	pose.theta = model_pose_.theta;
	pub_human_pose_.publish(pose);

	pose.x = model_robot_pose_.x;
	pose.y = model_robot_pose_.y;
	pose.theta = model_robot_pose_.theta;
	pub_robot_pose_.publish(pose);
}

void HumanModel::newGoalCallback(const human_sim::Goal::ConstPtr& goal)
{
	if(goal->x != current_goal_.x 
	|| goal->y != current_goal_.y 
	|| goal->theta != current_goal_.theta)
	{
		previous_goal_.x = 	current_goal_.x;
		previous_goal_.y = 	current_goal_.y;
		previous_goal_.theta = 	current_goal_.theta;

		current_goal_.x = 	goal->x;
		current_goal_.y = 	goal->y;
		current_goal_.theta = 	goal->theta;

		printf("CB current_goal = %f,%f\n", current_goal_.x, current_goal_.y);
		printf("CB previous_goal = %f,%f\n", previous_goal_.x, previous_goal_.y);
	}
}

void HumanModel::setBehaviorCallback(const std_msgs::Int32::ConstPtr& msg)
{
	bool changed=false;
	switch(msg->data)
	{
		case NONE:
			printf("Behavior set : NONE\n");
			changed=true;
			break;
		case RANDOM:
			printf("Behavior set : RANDOM\n");
			changed=true;
			break;
		case STOP_LOOK:
			printf("Behavior set : STOP_LOOK\n");
			changed=true;
			break;
		case HARASS:
			printf("Behavior set : HARASS\n");
			changed=true;
			break;

		default:
			break;
	}

	if(changed)
	{
		behavior_ = (Behavior)msg->data;
		sub_stop_look_ = WAIT_ROBOT;
		sub_harass_ = INIT;
	}
}

void HumanModel::newRandomGoalGeneration()
{
	if(ros::Time::now()-last_time_> delay_think_about_new_goal_)
	{
		int nb = rand()%100 + 1;
		printf("Tirage %d/%d\n", nb, chance_decide_new_goal_);
		if(nb < chance_decide_new_goal_)
		{
			printf("DECIDE NEW GOAL ! \n");
			human_sim::Goal previous_goal = current_goal_;
			human_sim::Goal new_goal = this->chooseGoal(true);
			if(new_goal.x != previous_goal.x || new_goal.y != previous_goal.y)
			{
				pub_new_goal_.publish(new_goal);
				printf("published\n");
			}
			else
				printf("ALREADY GOING!\n");
		}
		last_time_=ros::Time::now();
	}
}

void HumanModel::stopLookRobot()
{
	switch(sub_stop_look_)
	{
		case WAIT_ROBOT:
			printf("threshold=%f dist=%f\n", dist_near_robot_, sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2)));
			if(sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2))<dist_near_robot_)
				sub_stop_look_=STOP;
			break;

		case STOP:
			{
				printf("current_goal = %f,%f\n", current_goal_.x, current_goal_.y);
				printf("previous_goal = %f,%f\n", previous_goal_.x, previous_goal_.y);

				// Stop goal and motion
				printf("Stopped !\n");
				human_sim::CancelGoalAndStop srv_cancel;
				client_cancel_goal_and_stop_.call(srv_cancel);
	
				// Check if was in AUTONOMOUS
				human_sim::GetChoiceGoalDecision srv;
				if(client_get_choice_goal_decision_.call(srv))
					was_in_autonomous_ = srv.response.decision == 0;
				else
					ROS_ERROR("Failed to call get choice goal decision service!");
				printf("response = %d\n", srv.response.decision);
				if(was_in_autonomous_)
					printf("YES\n");
				else
					printf("NO!\n");

				// Passe en SPECIFIED
				std_msgs::Int32 msg;
				msg.data=1;
				pub_op_mode_.publish(msg);

				// Set global FSM to GET_GOAL
				human_sim::SetGetGoal srv_set;
				client_set_get_goal_.call(srv_set);

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

				printf("alpha=%f\n", alpha);//*180/PI);

				geometry_msgs::Twist cmd;
				if(abs(alpha-model_pose_.theta)>0.1)
				{
					cmd.angular.z=2;
					if(alpha-model_pose_.theta<0) 	
						cmd.angular.z=-cmd.angular.z;

					if(abs(alpha-model_pose_.theta)>PI)
						cmd.angular.z=-cmd.angular.z;
				}
				pub_perturbated_cmd_.publish(cmd);

				printf("threshold=%f dist=%f\n", dist_near_robot_*1.1, sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2)));
				if(sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2))>dist_near_robot_*1.1)
				{
					printf("NEW GOALLLLL\n");
					sub_stop_look_=NEW_GOAL;
				}
				break;
			}

		case NEW_GOAL:
			printf("Choose new goal\n");
			// resume current goal
			pub_new_goal_.publish(current_goal_);
			// put back in AUTONOMOUS if it was 
			if(was_in_autonomous_)
			{
				static ros::Time start = ros::Time::now();
				if(ros::Time::now() - start > ros::Duration(1))
				{
					printf("AUTO SENT!\n");
					std_msgs::Int32 msg;
					msg.data=0;
					pub_op_mode_.publish(msg);

					sub_stop_look_=OVER;
				}
			}
			else
				sub_stop_look_=OVER;
			break;

		case OVER:
			break;

		default:
			sub_stop_look_=WAIT_ROBOT;
			break;
	}
}

void HumanModel::harassRobot()
{
	switch(sub_harass_)
	{
		case INIT:
			{
				human_sim::CancelGoalAndStop srv_cancel;
				client_cancel_goal_and_stop_.call(srv_cancel);

				human_sim::SetGetGoal srv_set;
				client_set_get_goal_.call(srv_set);
				sub_harass_=HARASSING;
				break;
			}
		case HARASSING:
			if(ros::Time::now() > last_harass_ + delay_harass_replan_)
			{
				Pose2D in_front;
				in_front.x = model_robot_pose_.x + cos(model_robot_pose_.theta)*dist_in_front_;
				in_front.y = model_robot_pose_.y + sin(model_robot_pose_.theta)*dist_in_front_;
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

void HumanModel::behaviors()
{
	switch(behavior_)
	{
		case NONE:
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
			behavior_=NONE;
			break;
	}
}

void HumanModel::pubDist()
{
	float dist = sqrt(pow(model_robot_pose_.x-model_pose_.x,2) + pow(model_robot_pose_.y-model_pose_.y,2));
	std_msgs::String msg;
	msg.data = "HUMAN_MODEL DIST " + std::to_string(dist) + " " + std::to_string(ros::Time::now().toSec());
	pub_log_.publish(msg);
}

////////////////////////// MAIN ///////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	HumanModel human_model;

	ros::Rate rate(15);

	while(ros::ok() && (!hcb || !rcb))
	{
		ros::spinOnce();
		rate.sleep();
	}

	printf("LETS_GO\n");

	while(ros::ok())
	{
		// process data from simu
		human_model.processSimData();

		// Publish data as perceived by the human model
		human_model.publishModelData();

		// Add perturbation in human behaviors
		human_model.behaviors();

		// Publish human/robot distance to log
		human_model.pubDist();

		rate.sleep();
		ros::spinOnce();
	}
}

///////////////////////////////////////////////////////////

#include "humanModel.h"

bool hcb=false;
bool rcb=false;

/////////////////////// HUMAN MODEL ///////////////////////

HumanModel::HumanModel(ros::NodeHandle nh)
{
	nh_ = nh;

	srand(time(NULL));

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

	sub_pose_ = 	 	nh_.subscribe("sim/human_pose", 100, &HumanModel::poseCallback, this);
	sub_robot_pose_ =	nh_.subscribe("sim/robot_pose", 100, &HumanModel::robotPoseCallback, this);
	sub_cmd_geo_ =		nh_.subscribe("cmd_geo", 100, &HumanModel::cmdGeoCallback, this);
	sub_goal_done_ =	nh_.subscribe("goal_done", 100, &HumanModel::goalDoneCallback, this);

	pub_new_goal_ = 	nh_.advertise<human_sim::Goal>("boss/new_goal", 100);
	pub_human_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/human_pose", 100);
	pub_robot_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/robot_pose", 100);
	pub_perturbated_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("controller/perturbated_cmd", 100);
	pub_cancel_goal_ =	nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 100);

	service_ = 		nh_.advertiseService("choose_goal", &HumanModel::chooseGoalSrv, this);

	printf("I am human\n");


	// PARAMETERS//
	
	// Perturbation ratio
	//ratio_perturbation_ = 0.2; // +/- 20% of value
	ratio_perturbation_ = 0;

	// Chance to find a new random goal
	chance_decide_new_goal_ = 0; //x%
	delay_think_about_new_goal_ = ros::Duration(2); // x% chance every 2s
	last_time_ = ros::Time::now();

	// Behaviors
	behavior_ = STOP_NEAR;
	sub_stop_near_ = WAIT_ROBOT;

	// Near Robot distance
	dist_near_robot_ = 2;

	// INIT GOALS //
	human_sim::Goal goal;
	GoalArea area;
	area.goal.type="Position";

	area.goal.x=0.9; 	area.goal.y=0.9; 	area.goal.theta=-PI/2;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=3.15; 	area.goal.y=3.6; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=10.51; 	area.goal.y=-3.98; 	area.goal.theta=0;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=8.07; 	area.goal.y=5.07; 	area.goal.theta=-PI/2;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=7.8; 	area.goal.y=9.98; 	area.goal.theta=-PI;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=3.42; 	area.goal.y=9.48; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=4.72; 	area.goal.y=17.68; 	area.goal.theta=PI/2;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=10.71; 	area.goal.y=15.8; 	area.goal.theta=0;	area.radius=0;
	known_goals_.push_back(area);
	area.goal.x=1.15; 	area.goal.y=6.52; 	area.goal.theta=-PI;	area.radius=0;
	known_goals_.push_back(area);

	area.goal.x=8.8; 	area.goal.y=0.8; 	area.goal.theta=0;	area.radius=1.3;
	known_goals_.push_back(area);
	area.goal.x=3.0; 	area.goal.y=15.3; 	area.goal.theta=0;	area.radius=2;
	known_goals_.push_back(area);	
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
	human_sim::Goal goal = this->chooseGoal();

	res.goal.type = 	goal.type;
	res.goal.x = 		goal.x;
	res.goal.y = 		goal.y;
	res.goal.theta = 	goal.theta;
}

human_sim::Goal HumanModel::chooseGoal()
{
	human_sim::Goal goal;

/*	int i;
	do
	{
		i=rand()%known_goals_.size();
	}while(known_goals_[i].x==previous_goal_.x && known_goals_[i].y==previous_goal_.y);*/

	static int i=-1;
	i=(i+1)%known_goals_.size();

	printf("i=%d\n", i);
	printf("x=%f, y=%f, theta=%f, radius=%f\n", known_goals_[i].goal.x,known_goals_[i].goal.y,known_goals_[i].goal.theta,known_goals_[i].radius);

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

void HumanModel::newRandomGoalGeneration(bool toss)
{
	if(!toss || ros::Time::now()-last_time_> delay_think_about_new_goal_)
	{
		int nb = rand()%100 + 1;
		if(!toss || nb < chance_decide_new_goal_)
		{
			printf("DECIDE NEW GOAL ! \n");
			human_sim::Goal previous_goal = current_goal_;
			human_sim::Goal new_goal = this->chooseGoal();
			if(new_goal.x != previous_goal.x || new_goal.y != previous_goal.y)
			{
				pub_new_goal_.publish(this->chooseGoal());
				printf("published\n");
			}
			else
				printf("ALREADY GOING!\n");
		}
		last_time_=ros::Time::now();
	}
}

void HumanModel::stopNearRobot()
{
	switch(sub_stop_near_)
	{
		case WAIT_ROBOT:
			printf("threshold=%f dist=%f\n", dist_near_robot_, sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2)));
			if(sqrt(pow(model_pose_.x-model_robot_pose_.x,2) + pow(model_pose_.y-model_robot_pose_.y,2))<dist_near_robot_)
				sub_stop_near_=STOP;
			break;

		case STOP:
			printf("Stopped !\n");
			pub_cancel_goal_.publish(actionlib_msgs::GoalID());
			sub_stop_near_=WAIT_AFTER;
			break;

		case WAIT_AFTER:
			if(abs(model_pose_.x-model_robot_pose_.x)>=dist_near_robot_
			|| abs(model_pose_.y-model_robot_pose_.y)>=dist_near_robot_)
				sub_stop_near_=NEW_GOAL;
			break;

		case NEW_GOAL:
			printf("Choose new goal\n");
			this->newRandomGoalGeneration(false); // goal behind
			sub_stop_near_=OVER;
			break;

		case OVER:
			break;

		default:
			sub_stop_near_=WAIT_ROBOT;
	}
}

void HumanModel::behaviors()
{
	switch(behavior_)
	{
		case NONE:
			break;

		case RANDOM:
			this->newRandomGoalGeneration(true);
			break;

		case STOP_NEAR:
			this->stopNearRobot();
			break;

		default:
			behavior_=NONE;
			break;
	}
}

////////////////////////// MAIN ///////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	ros::NodeHandle nh;

	HumanModel human_model(nh);

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

		rate.sleep();
		ros::spinOnce();
	}
}

///////////////////////////////////////////////////////////

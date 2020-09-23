#include "humanModel.h"


/////////////////////////// MAP ///////////////////////////

Map::Map(float rx, float ry, float tile_size)
{
	real_size_x_ = rx;
	real_size_y_ = ry;
	tile_size_ = tile_size;

	int nx = real_size_x_/tile_size_;
	int ny = real_size_y_/tile_size_;

	std::vector<Tile> column;
	for(int i=0; i<nx; i++)
	{
		column.clear();
		for(int j=0; j<ny; j++)
			column.push_back(FREE);
		map_.push_back(column);
	}

	map_[7][1]=GOAL;

	map_[7][5]=GOAL;
}

void Map::show()
{
	printf("\n");
	if(map_.size()>0)
	{
		for(int j=0; j<map_[0].size(); j++)
		{
			for(int i=0; i<map_.size(); i++)
				printf("%d ", map_[i][j]);
			printf("\n");
		}
	}
	printf("\n");
}

/////////////////////// HUMAN MODEL ///////////////////////

HumanModel::HumanModel(ros::NodeHandle nh): map_(10, 10, 1)
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

	last_cmd_geo_.linear.x=0;
	last_cmd_geo_.linear.y=0;
	last_cmd_geo_.linear.z=0;
	last_cmd_geo_.angular.x=0;
	last_cmd_geo_.angular.y=0;
	last_cmd_geo_.angular.z=0;

	sub_pose_ = 	 	nh_.subscribe("sim/human_pose", 100, &HumanModel::poseCallback, this);
	sub_robot_pose_ =	nh_.subscribe("sim/robot_pose", 100, &HumanModel::robotPoseCallback, this);
	sub_cmd_geo_ =		nh_.subscribe("cmd_geo", 100, &HumanModel::cmdGeoCallback, this);
	sub_goal_done_ =	nh_.subscribe("goal_done", 100, &HumanModel::goalDoneCallback, this);

	pub_new_goal_ = 	nh_.advertise<human_sim::Goal>("boss/new_goal", 100);
	pub_human_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/human_pose", 100);
	pub_robot_pose_ = 	nh_.advertise<geometry_msgs::Pose2D>("human_model/robot_pose", 100);
	pub_perturbated_cmd_ = 	nh_.advertise<geometry_msgs::Twist>("controller/perturbated_cmd", 100);

	service_ = nh_.advertiseService("choose_goal", &HumanModel::chooseGoal, this);

	printf("I am human\n");

	map_.show();
}

void HumanModel::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_pose_.x=msg->x;
	sim_pose_.y=msg->y;
	sim_pose_.theta=msg->theta;
}

void HumanModel::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	sim_robot_pose_.x=msg->x;
	sim_robot_pose_.y=msg->y;
	sim_robot_pose_.theta=msg->theta;
}

void HumanModel::cmdGeoCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist perturbated_cmd;

	// add perturbation using the last cmd eg: (cmd-last)/deltaT*ratio + cmd
	// for now no perturbation
	perturbated_cmd.linear.x=msg->linear.x;
	perturbated_cmd.linear.y=msg->linear.y;
	perturbated_cmd.linear.z=msg->linear.z;
	perturbated_cmd.angular.x=msg->angular.x;
	perturbated_cmd.angular.y=msg->angular.y;
	perturbated_cmd.angular.z=msg->angular.z;

	// publish noisy cmd
	pub_perturbated_cmd_.publish(perturbated_cmd);
}

void HumanModel::goalDoneCallback(const human_sim::Goal::ConstPtr& msg)
{
	printf("received !!\n");
	if(msg->type=="Position")
	{
		printf("REMOVE GOAL\n");
		map_.map_[map_.getNX()/2-(int)msg->y][map_.getNY()/2-(int)msg->x]=Map::FREE;
	}
}

bool HumanModel::chooseGoal(human_sim::ChooseGoal::Request& req, human_sim::ChooseGoal::Response& res)
{
	map_.show();

	// search for goals in the map
	int x(-1),y(-1);
	for(int i=0; i<map_.getNX(); i++)
	{
		for(int j=0; j< map_.getNY(); j++)
		{
			if(map_.map_[i][j]==Map::GOAL)
			{
				x=i;
				y=j;
			}
		}
	}

	res.goal.type =	 "Position"; // only choice for now
	res.goal.theta = (rand()%30)/10.0;
	if(x!=-1 && y!=-1)
	{
		printf("x=%d y=%d\n", x, y);
		res.goal.x = map_.getNX()/2-y;
		res.goal.y = map_.getNY()/2-x;
	}
	else
	{
		// randomly pick a position as a goal
		res.goal.x = 	(rand()%100)/10.0;
		res.goal.y = 	(rand()%100)/10.0;
	}

	printf("goal choosen !\n");
	printf("%s (%f, %f, %f)\n", res.goal.type.c_str(), res.goal.x, res.goal.y, res.goal.theta);
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

////////////////////////// MAIN ///////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "human_model");

	ros::NodeHandle nh;

	HumanModel human_model(nh);

	ros::Rate rate(5);

	while(ros::ok())
	{
		// process data from simu
		// -> memory etc..
		human_model.processSimData();

		// Publish data as perceived by the human model
		human_model.publishModelData();

		rate.sleep();
		ros::spinOnce();
	}
}

///////////////////////////////////////////////////////////

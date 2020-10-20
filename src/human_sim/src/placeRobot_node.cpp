#include "placeRobot.h"

///////////////////// PLACE ROBOT MAP //////////////////////

PlaceRobotMap::PlaceRobotMap(ros::NodeHandle nh)
{
	ROS_INFO("DEBUT constu\n");
	nh_=nh;

	robot_pose_.x = 	0;
	robot_pose_.y = 	0;
	robot_pose_.theta = 	0;

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	size_rob_ = 4;
	dist_threshold_ = 5/0.05;
	robot_near_ = false;

	map_initiated_ = false;

	robot_pose_sub_ = 	nh_.subscribe("human_model/robot_pose", 100, &PlaceRobotMap::robotPoseCallback, this);
	human_pose_sub_ = 	nh_.subscribe("human_model/human_pose", 100, &PlaceRobotMap::humanPoseCallback, this);
	map_sub_ = 		nh_.subscribe("map", 100, &PlaceRobotMap::mapCallback, this);

	map_pub_ = 		nh_.advertise<map_msgs::OccupancyGridUpdate>("map_updates", 100);

	ROS_INFO("constructeur fini\n");
}

void PlaceRobotMap::computeAndPublish()
{
	if(map_initiated_)
	{
		if(sqrt(pow(human_pose_.x-robot_pose_.x,2) + pow(human_pose_.y-robot_pose_.y,2)) < dist_threshold_)
		{
			robot_near_ = true;

			map_msgs::OccupancyGridUpdate new_map;
			new_map.x = 	0;
			new_map.y = 	0;
			new_map.width = map_.info.width;
			new_map.height=	map_.info.height;
			new_map.data = 	map_.data;

			for(int i=robot_pose_.x-size_rob_; i<robot_pose_.x+size_rob_; i++)
			{
				for(int j=robot_pose_.y-size_rob_; j<robot_pose_.y+size_rob_; j++)
				{
					new_map.data[j*new_map.width + i] = 100;
				}
			}

			map_pub_.publish(new_map);
		}
		else if(robot_near_)
		{
			robot_near_ = false;

			map_msgs::OccupancyGridUpdate new_map;
			new_map.x = 	0;
			new_map.y = 	0;
			new_map.width = map_.info.width;
			new_map.height=	map_.info.height;
			new_map.data = 	map_.data;

			map_pub_.publish(new_map);
		}
	}
}

void PlaceRobotMap::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose)
{
	robot_pose_.x = 	(r_pose->x+2)/0.05; // resol map
	robot_pose_.y = 	(r_pose->y+8)/0.05;
	robot_pose_.theta = 	r_pose->theta;
}

void PlaceRobotMap::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	human_pose_.x = 	(h_pose->x+2)/0.05; // resol map
	human_pose_.y = 	(h_pose->y+8)/0.05;
	human_pose_.theta = 	h_pose->theta;
}

void PlaceRobotMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	map_ = *map;
	map_initiated_ = true;

	printf("width=%d\n", map_.info.width);
	printf("height=%d\n", map_.info.height);

	printf("MAP received \n");
}

//////////////////////// MAIN //////////////////////////////

int main(int argc, char** argv)
{
	ROS_INFO("=> MAIN place robot\n");

	ros::init(argc, argv, "place_robot");

	ros::NodeHandle nh;

	PlaceRobotMap place_robot_map(nh);

	ros::Rate loop(25);

	while(ros::ok())
	{
		place_robot_map.computeAndPublish();
		ros::spinOnce();

		loop.sleep();
	}

	return 0;
}

////////////////////////////////////////////////////////////

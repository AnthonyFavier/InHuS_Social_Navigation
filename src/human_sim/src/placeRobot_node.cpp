#include "ros/ros.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "type.h"

class PlaceRobotMap
{
public:
	PlaceRobotMap(ros::NodeHandle nh)
	{
		nh_=nh;

		robot_pose_.x = 	0;
		robot_pose_.y = 	0;
		robot_pose_.theta = 	0;

		size_rob_ = 5;

		map_initiated_ = false;

		robot_pose_sub_ = 	nh_.subscribe("human_model/robot_pose", 100, &PlaceRobotMap::robotPoseCallback, this);
		map_sub_ = 		nh_.subscribe("map", 100, &PlaceRobotMap::mapCallback, this);

		map_pub_ = 		nh_.advertise<map_msgs::OccupancyGridUpdate>("map_updates", 100);

		printf("constructeur fini\n");
	}


	void computeAndPublish()
	{
		if(map_initiated_) // and if robot not too far
		{
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
	}

private:
	ros::NodeHandle nh_;

	ros::Subscriber robot_pose_sub_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose)
	{
		robot_pose_.x = 	(r_pose->x+2)/0.05; // resol map
		robot_pose_.y = 	(r_pose->y+8)/0.05;
		robot_pose_.theta = 	r_pose->theta;

	}
	Pose2D robot_pose_;

	ros::Subscriber map_sub_;
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
	{
		map_ = *map;
		map_initiated_ = true;

		printf("width=%d\n", map_.info.width);
		printf("height=%d\n", map_.info.height);

		printf("MAP received \n");
	}
	nav_msgs::OccupancyGrid map_;
	bool map_initiated_;

	ros::Publisher map_pub_;

	int size_rob_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_robot");

	ros::NodeHandle nh;

	PlaceRobotMap place_robot_map(nh);

	ros::Rate loop(10);

	while(ros::ok())
	{
		place_robot_map.computeAndPublish();
		ros::spinOnce();

		loop.sleep();
	}

	return 0;
}

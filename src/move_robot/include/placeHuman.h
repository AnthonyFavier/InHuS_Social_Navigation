#ifndef DEF_PLACE_HUMAN_MAP
#define DEF_PLACE_HUMAN_MAP

#include "ros/ros.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"

struct Pose2D
{
	float x;
	float y;
	float theta;
};

class PlaceHumanMap
{
public:
	PlaceHumanMap();
	void computeAndPublish();

private:
	ros::NodeHandle nh_;
	
	ros::Subscriber robot_pose_sub_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose);
	Pose2D robot_pose_;

	ros::Subscriber human_pose_sub_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	Pose2D human_pose_;

	ros::Subscriber map_sub_;
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
	nav_msgs::OccupancyGrid map_;
	bool map_initiated_;

	ros::Publisher map_pub_;

	int size_rob_;
	float dist_threshold_;
	bool human_near_;
};

#endif

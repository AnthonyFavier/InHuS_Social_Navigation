#ifndef DEF_PLACE_HUMAN_MAP
#define DEF_PLACE_HUMAN_MAP

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

class PlaceHumanMap
{
public:
	PlaceHumanMap();

	bool initDone();

	void start();

	void placeRobot();

private:
	ros::NodeHandle nh_;
	
	ros::Subscriber human_pose_sub_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	geometry_msgs::Pose2D human_pose_;
	bool hcb_;

	bool active_;

	sensor_msgs::PointCloud2 human_pose_PointCloud2_;
	sensor_msgs::PointCloud2 empty_PointCloud2_;
	ros::Publisher human_pose_pub_;

	float size_h_;
};

#endif

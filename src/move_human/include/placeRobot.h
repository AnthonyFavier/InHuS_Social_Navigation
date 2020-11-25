#ifndef DEF_PLACE_ROBOT_MAP
#define DEF_PLACE_ROBOT_MAP

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "type.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

class PlaceRobotMap
{
public:
	PlaceRobotMap();

private:
	ros::NodeHandle nh_;
	
	ros::Subscriber robot_pose_sub_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose);
	Pose2D robot_pose_;

	ros::Subscriber human_pose_sub_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	Pose2D human_pose_;
	bool human_pose_init_;

	sensor_msgs::PointCloud2 robot_pose_PointCloud2_;
	sensor_msgs::PointCloud2 empty_PointCloud2_;
	ros::Publisher robot_pose_pub_;

	const float size_rob_;
	const float dist_threshold_;
	bool robot_near_;
};

#endif

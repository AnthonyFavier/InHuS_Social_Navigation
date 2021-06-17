#ifndef DEF_PLACE_ROBOT_MAP
#define DEF_PLACE_ROBOT_MAP

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "inhus_navigation/PlaceRobot.h"
#include "std_srvs/SetBool.h"

class PlaceRobotMap
{
public:
	PlaceRobotMap();
	bool placeRobotSrv(inhus_navigation::PlaceRobot::Request& req, inhus_navigation::PlaceRobot::Response& res);

	bool initDone();

	void start();

	void placeRobot();

private:
	ros::NodeHandle nh_;

	ros::Subscriber robot_pose_sub_;
	void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose);
	geometry_msgs::Pose2D robot_pose_;
	bool rcb_;

	ros::Subscriber human_pose_sub_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	geometry_msgs::Pose2D human_pose_;
	bool hcb_;

	bool active_;

	sensor_msgs::PointCloud2 robot_pose_PointCloud2_;
	sensor_msgs::PointCloud2 empty_PointCloud2_;
	ros::Publisher robot_pose_pub_;

	ros::ServiceServer place_robot_server_;
	ros::ServiceClient client_shutdown_layer_static_human_;
	ros::ServiceClient client_shutdown_layer_visible_human_;

	float size_rob_;
	float dist_threshold_;

	bool robot_near_;
	bool place_robot_;
};

#endif

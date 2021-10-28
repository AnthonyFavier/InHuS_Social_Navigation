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
#include "inhus/PoseVel.h"

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

	ros::Subscriber r_pose_vel_sub_;
	void rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);
	inhus::PoseVel r_pose_vel_;
	bool rcb_;

	ros::Subscriber h_pose_vel_sub_;
	void hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg);
	inhus::PoseVel h_pose_vel_;
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

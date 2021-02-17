#ifndef DEF_PLACE_OTHER_MAP
#define DEF_PLACE_OTHER_MAP

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "move_human/PlaceOther.h"

class PlaceOtherMap
{
public:
	PlaceOtherMap();
	bool placeOtherSrv(move_human::PlaceOther::Request& req, move_human::PlaceOther::Response& res);


	bool initDone();

	void start();

	void placeOther();

private:
	ros::NodeHandle nh_;

	ros::Subscriber human_pose_sub_;
	void humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	geometry_msgs::Pose2D human_pose_;
	bool hcb_;

	ros::Subscriber other_pose_sub_;
	void otherPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose);
	geometry_msgs::Pose2D other_pose_;
	bool ocb_;

	bool active_;

	sensor_msgs::PointCloud2 other_pose_PointCloud2_;
	sensor_msgs::PointCloud2 empty_PointCloud2_;
	ros::Publisher other_pose_pub_;

	ros::ServiceServer place_other_server_;

	float size_h_;
	float dist_threshold_;
	bool place_other_;
};

#endif

#include "placeHuman.h"

///////////////////// PLACE HUMAN MAP //////////////////////

PlaceHumanMap::PlaceHumanMap()
{
	// Ros Params
	size_h_ = 0.6;

	// Init
	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	hcb_ = 	false;

	active_ = false;

	// Build empty_PointCloud2
	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "human_human_base";
	cloud.header.stamp = ros::Time::now();
	sensor_msgs::convertPointCloudToPointCloud2(cloud, empty_PointCloud2_);

	// Build human_pose_PoinCloud2_
	cloud.header.frame_id = "human_human_base";
	cloud.header.stamp = ros::Time::now();
	geometry_msgs::Point32 point;
	point.z = 0.0;
	point.x = -size_h_/2;
	point.y = -size_h_/2;
	cloud.points.push_back(point);	
	point.x = -size_h_/2;
	point.y = 0.01;
	cloud.points.push_back(point);
	point.x = -size_h_/2;
	point.y = size_h_/2;
	cloud.points.push_back(point);
	point.x = 0;
	point.y = size_h_/2;
	cloud.points.push_back(point);
	point.x = size_h_/2;
	point.y = size_h_/2;
	cloud.points.push_back(point);
	point.x = size_h_/2;
	point.y = 0;
	cloud.points.push_back(point);
	point.x = size_h_/2;
	point.y = -size_h_/2;
	cloud.points.push_back(point);
	point.x = 0;
	point.y = -size_h_/2;
	cloud.points.push_back(point);
	sensor_msgs::convertPointCloudToPointCloud2(cloud, human_pose_PointCloud2_);

	// Subscriber
	human_pose_sub_ = 	nh_.subscribe("known/human_other_pose", 100, &PlaceHumanMap::humanPoseCallback, this);

	// Publisher
	human_pose_pub_ =	nh_.advertise<sensor_msgs::PointCloud2>("human_pose_PointCloud2", 10);
}

void PlaceHumanMap::placeHuman()
{
	// publish with obst
	human_pose_PointCloud2_.header.stamp = ros::Time::now();
	human_pose_pub_.publish(human_pose_PointCloud2_);
}

bool PlaceHumanMap::initDone()
{
	return hcb_;
}

void PlaceHumanMap::start()
{
	active_ = true;
}

void PlaceHumanMap::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	human_pose_.x = 	h_pose->x+2;
	human_pose_.y = 	h_pose->y+8;
	human_pose_.theta = 	h_pose->theta;
	hcb_ =	true;

	if(active_)
		this->placeHuman();
}

//////////////////////// MAIN //////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_human");

	PlaceHumanMap place_human_map;

	ros::Rate loop(60);

	// wait init
	ROS_INFO("Waiting for init ...");
	while(ros::ok() && !place_human_map.initDone())
	{
		ros::spinOnce();
		loop.sleep();
	}
	ROS_INFO("INIT done");

	place_human_map.start();

	ros::spin();

	return 0;
}

////////////////////////////////////////////////////////////


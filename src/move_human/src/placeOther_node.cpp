#include "placeOther.h"

///////////////////// PLACE HUMAN MAP //////////////////////

PlaceOtherMap::PlaceOtherMap()
{
	// Ros Params
	ros::NodeHandle private_nh("~");
	private_nh.param(std::string("size_h"), size_h_, float(0.6));
	private_nh.param(std::string("dist_threshold"), dist_threshold_, float(5.0));

	ROS_INFO("Params:");
	ROS_INFO("size_h=%f", size_h_);
	ROS_INFO("dist_threshold=%f", dist_threshold_);

	// Init
	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	other_pose_.x = 	0;
	other_pose_.y = 	0;
	other_pose_.theta = 	0;

	ocb_ = 	false;
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
	sensor_msgs::convertPointCloudToPointCloud2(cloud, other_pose_PointCloud2_);

	// Subscriber
	other_pose_sub_ = 	nh_.subscribe("known/other_pose", 100, &PlaceOtherMap::otherPoseCallback, this);
	human_pose_sub_ =		nh_.subscribe("known/human_pose", 100, &PlaceOtherMap::humanPoseCallback, this);

	// Publisher
	other_pose_pub_ =	nh_.advertise<sensor_msgs::PointCloud2>("other_pose_PointCloud2", 10);

	// Server
	place_other_server_ = nh_.advertiseService("place_other", &PlaceOtherMap::placeOtherSrv, this);
}

bool PlaceOtherMap::placeOtherSrv(move_human::PlaceOther::Request& req, move_human::PlaceOther::Response& res)
{
	place_other_ = req.data;

	this->placeOther();

	return true;
}

void PlaceOtherMap::placeOther()
{
	if(place_other_)
	{
		// check if also not too far
		if(sqrt(pow(other_pose_.x-human_pose_.x,2) + pow(other_pose_.y-human_pose_.y,2)) <= dist_threshold_)
		{
			// publish with obst
			other_pose_PointCloud2_.header.stamp = ros::Time::now();
			other_pose_pub_.publish(other_pose_PointCloud2_);
		}
		else
		{
			// publish empty
			empty_PointCloud2_.header.stamp = ros::Time::now();
			other_pose_pub_.publish(empty_PointCloud2_);
		}
	}
	else
	{
		// publish empty
		empty_PointCloud2_.header.stamp = ros::Time::now();
		other_pose_pub_.publish(empty_PointCloud2_);
	}
}

bool PlaceOtherMap::initDone()
{
	return hcb_ && ocb_;
}

void PlaceOtherMap::start()
{
	active_ = true;
}

void PlaceOtherMap::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	human_pose_.x = 	h_pose->x+2;
	human_pose_.y = 	h_pose->y+8;
	human_pose_.theta = 	h_pose->theta;
	hcb_ =	true;
}

void PlaceOtherMap::otherPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	other_pose_.x = 	h_pose->x+2;
	other_pose_.y = 	h_pose->y+8;
	other_pose_.theta = 	h_pose->theta;
	ocb_ =	true;

	if(active_)
		this->placeOther();
}

//////////////////////// MAIN //////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_other");

	PlaceOtherMap place_other_map;

	ros::Rate loop(60);

	// wait init
	ROS_INFO("Waiting for init ...");
	while(ros::ok() && !place_other_map.initDone())
	{
		ros::spinOnce();
		loop.sleep();
	}
	ROS_INFO("INIT done");

	place_other_map.start();

	ros::spin();

	return 0;
}

////////////////////////////////////////////////////////////

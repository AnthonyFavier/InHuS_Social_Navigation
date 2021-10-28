#include "placeRobot.h"

///////////////////// PLACE ROBOT MAP //////////////////////

PlaceRobotMap::PlaceRobotMap()
{
	// Ros Params
	ros::NodeHandle private_nh("~");
	private_nh.param(std::string("size_rob"), size_rob_, float(0.6));
	private_nh.param(std::string("dist_threshold"), dist_threshold_, float(5.0));

	ROS_INFO("=> Params PlaceRobot :");
	ROS_INFO("size_rob=%f", size_rob_);
	ROS_INFO("dist_threshold=%f", dist_threshold_);

	// Init
	place_robot_ = false;

	hcb_ = 	false;
	rcb_ =  false;

	active_ = false;

	// Build empty_PointCloud2
	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "robot_inhus";
	cloud.header.stamp = ros::Time::now();
	sensor_msgs::convertPointCloudToPointCloud2(cloud, empty_PointCloud2_);

	// Build robot_pose_PoinCloud2_
	geometry_msgs::Point32 point;
	point.z = 0.0;
	point.x = -size_rob_/2;
	point.y = -size_rob_/2;
	cloud.points.push_back(point);
	point.x = -size_rob_/2;
	point.y = 0.01;
	cloud.points.push_back(point);
	point.x = -size_rob_/2;
	point.y = size_rob_/2;
	cloud.points.push_back(point);
	point.x = 0;
	point.y = size_rob_/2;
	cloud.points.push_back(point);
	point.x = size_rob_/2;
	point.y = size_rob_/2;
	cloud.points.push_back(point);
	point.x = size_rob_/2;
	point.y = 0;
	cloud.points.push_back(point);
	point.x = size_rob_/2;
	point.y = -size_rob_/2;
	cloud.points.push_back(point);
	point.x = 0;
	point.y = -size_rob_/2;
	cloud.points.push_back(point);
	cloud.header.stamp = ros::Time::now();
	sensor_msgs::convertPointCloudToPointCloud2(cloud, robot_pose_PointCloud2_);

	// Subscriber
	r_pose_vel_sub_ = 	nh_.subscribe("known/robot_pose_vel", 100, &PlaceRobotMap::rPoseVelCallback, this);
	h_pose_vel_sub_ = 	nh_.subscribe("known/human_pose_vel", 100, &PlaceRobotMap::hPoseVelCallback, this);

	// Publisher
	robot_pose_pub_ =	nh_.advertise<sensor_msgs::PointCloud2>("robot_pose_PointCloud2", 10);

	// Server
	place_robot_server_ = nh_.advertiseService("place_robot", &PlaceRobotMap::placeRobotSrv, this);
}

bool PlaceRobotMap::placeRobotSrv(inhus_navigation::PlaceRobot::Request& req, inhus_navigation::PlaceRobot::Response& res)
{
	place_robot_ = req.data;

	if(active_)
		this->placeRobot();

	return true;
}

void PlaceRobotMap::placeRobot()
{
	if(place_robot_)
	{
		// check if also not too far
		if(sqrt(pow(h_pose_vel_.pose.x-r_pose_vel_.pose.x,2) + pow(h_pose_vel_.pose.y-r_pose_vel_.pose.y,2)) <= dist_threshold_)
		{
			// publish with obst
			robot_pose_PointCloud2_.header.stamp = ros::Time::now();
			robot_pose_pub_.publish(robot_pose_PointCloud2_);
		}
		else
		{
			// publish empty
			empty_PointCloud2_.header.stamp = ros::Time::now();
			robot_pose_pub_.publish(empty_PointCloud2_);
		}
	}
	else
	{
		// publish empty
		empty_PointCloud2_.header.stamp = ros::Time::now();
		robot_pose_pub_.publish(empty_PointCloud2_);
	}
}

bool PlaceRobotMap::initDone()
{
	return rcb_ && hcb_;
}

void PlaceRobotMap::rPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	r_pose_vel_ = 	*msg;
	rcb_ = true;

	if(active_)
		this->placeRobot();
}

void PlaceRobotMap::hPoseVelCallback(const inhus::PoseVel::ConstPtr& msg)
{
	h_pose_vel_ = 	*msg;
	hcb_ = true;
}

void PlaceRobotMap::start()
{
	active_ = true;
}

//////////////////////// MAIN //////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_robot");

	PlaceRobotMap place_robot_map;

	ros::Rate rate(60);

	// wait init
	ROS_INFO("PlaceRobot: Waiting for init ...");
	while(ros::ok() && !place_robot_map.initDone())
	{
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("PlaceRobot: INIT done");

	place_robot_map.start();

	ros::spin();

	return 0;
}

////////////////////////////////////////////////////////////

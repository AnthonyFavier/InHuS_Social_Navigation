#include "placeRobot.h"

///////////////////// PLACE ROBOT MAP //////////////////////

PlaceRobotMap::PlaceRobotMap()
{
	// Ros Params
	ros::NodeHandle private_nh("~");
	private_nh.param(std::string("size_rob"), size_rob_, float(0.5));
	private_nh.param(std::string("dist_threshold"), dist_threshold_, float(5.0));

	ROS_INFO("Params:");
	ROS_INFO("size_rob=%f", size_rob_);
	ROS_INFO("dist_threshold=%f", dist_threshold_);

	// Init
	robot_pose_.x = 	0;
	robot_pose_.y = 	0;
	robot_pose_.theta = 	0;

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;

	human_pose_init_ = 	false;
	robot_near_ = false;

	// Build empty_PointCloud2
	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "human_robot_base";
	cloud.header.stamp = ros::Time::now();
	sensor_msgs::convertPointCloudToPointCloud2(cloud, empty_PointCloud2_);

	// Build robot_pose_PoinCloud2_
	cloud.header.frame_id = "human_robot_base";
	cloud.header.stamp = ros::Time::now();
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
	/*sensor_msgs::ChannelFloat32 channel;
	  channel.name = "intensity";
	  channel.values.push_back(100.0);
	  cloud.channels.push_back(channel);*/
	sensor_msgs::convertPointCloudToPointCloud2(cloud, robot_pose_PointCloud2_);

	// Subscriber
	robot_pose_sub_ = 	nh_.subscribe("known/robot_pose", 100, &PlaceRobotMap::robotPoseCallback, this);
	human_pose_sub_ = 	nh_.subscribe("known/human_pose", 100, &PlaceRobotMap::humanPoseCallback, this);

	// Publisher
	robot_pose_pub_ =	nh_.advertise<sensor_msgs::PointCloud2>("robot_pose_PointCloud2", 10);
}

void PlaceRobotMap::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose)
{
	if(human_pose_init_)
	{
		robot_pose_.x = 	r_pose->x+2;
		robot_pose_.y = 	r_pose->y+8;
		robot_pose_.theta = 	r_pose->theta;

		if(sqrt(pow(human_pose_.x-robot_pose_.x,2) + pow(human_pose_.y-robot_pose_.y,2)) < dist_threshold_)
		{
			robot_near_ = true;

			// publish with obst
			robot_pose_PointCloud2_.header.stamp = ros::Time::now();
			robot_pose_pub_.publish(robot_pose_PointCloud2_);

		}
		else if(robot_near_)
		{
			robot_near_ = false;

			// publish empty
			empty_PointCloud2_.header.stamp = ros::Time::now();
			robot_pose_pub_.publish(empty_PointCloud2_);
		}
	}
}

void PlaceRobotMap::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	human_pose_.x = 	h_pose->x+2;
	human_pose_.y = 	h_pose->y+8;
	human_pose_.theta = 	h_pose->theta;
	human_pose_init_ = 	true;
}

//////////////////////// MAIN //////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "place_robot");

	PlaceRobotMap place_robot_map;

	ros::spin();

	return 0;
}

////////////////////////////////////////////////////////////

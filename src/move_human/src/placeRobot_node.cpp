#include "placeRobot.h"

///////////////////// PLACE ROBOT MAP //////////////////////

PlaceRobotMap::PlaceRobotMap()
{
	robot_pose_.x = 	0;
	robot_pose_.y = 	0;
	robot_pose_.theta = 	0;

	human_pose_.x = 	0;
	human_pose_.y = 	0;
	human_pose_.theta = 	0;
	human_pose_init_ = 	false;

	size_rob_ = 6;
	dist_threshold_ = 4/0.05;
	robot_near_ = false;

	robot_pose_sub_ = 	nh_.subscribe("human_model/robot_pose", 100, &PlaceRobotMap::robotPoseCallback, this);
	human_pose_sub_ = 	nh_.subscribe("human_model/human_pose", 100, &PlaceRobotMap::humanPoseCallback, this);

	robot_pose_pub_ =	nh_.advertise<sensor_msgs::PointCloud2>("robot_pose_PointCloud2", 10);
}

void PlaceRobotMap::robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& r_pose)
{
	if(human_pose_init_)
	{
		robot_pose_.x = 	(r_pose->x+2)/0.05; // resol map
		robot_pose_.y = 	(r_pose->y+8)/0.05;
		robot_pose_.theta = 	r_pose->theta;

		if(sqrt(pow(human_pose_.x-robot_pose_.x,2) + pow(human_pose_.y-robot_pose_.y,2)) < dist_threshold_)
		{
			robot_near_ = true;

			// publish with obst
			sensor_msgs::PointCloud cloud;

			cloud.header.frame_id = "human_robot_base";
			cloud.header.stamp = ros::Time::now();

			geometry_msgs::Point32 point;
			point.z = 0.0;

			point.x = -0.25;
			point.y = -0.25;
			cloud.points.push_back(point);	
			point.x = -0.25;
			point.y = 0.01;
			cloud.points.push_back(point);
			point.x = -0.25;
			point.y = 0.25;
			cloud.points.push_back(point);
			point.x = 0;
			point.y = 0.25;
			cloud.points.push_back(point);
			point.x = 0.25;
			point.y = 0.25;
			cloud.points.push_back(point);
			point.x = 0.25;
			point.y = 0;
			cloud.points.push_back(point);
			point.x = 0.25;
			point.y = -0.25;
			cloud.points.push_back(point);
			point.x = 0;
			point.y = -0.25;
			cloud.points.push_back(point);

			/*sensor_msgs::ChannelFloat32 channel;
			  channel.name = "intensity";
			  channel.values.push_back(100.0);
			  cloud.channels.push_back(channel);*/

			sensor_msgs::PointCloud2 cloud2;
			sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

			robot_pose_pub_.publish(cloud2);

		}
		else if(robot_near_)
		{
			robot_near_ = false;

			// publish empty
			sensor_msgs::PointCloud cloud;

			cloud.header.frame_id = "human_robot_base";
			cloud.header.stamp = ros::Time::now();

			sensor_msgs::PointCloud2 cloud2;
			sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

			robot_pose_pub_.publish(cloud2);
		}
	}


}

void PlaceRobotMap::humanPoseCallback(const geometry_msgs::Pose2D::ConstPtr& h_pose)
{
	human_pose_.x = 	(h_pose->x+2)/0.05; // resol map
	human_pose_.y = 	(h_pose->y+8)/0.05;
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

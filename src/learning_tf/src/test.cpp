#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_tf_broadcaster");

	ros::NodeHandle node;

	ros::Rate rate(50);

	tf::TransformBroadcaster br;

	tf::Transform transform;
	transform.setOrigin( tf::Vector3(2, 2, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

	tf::Transform transform_laser;
	transform_laser.setOrigin( tf::Vector3(0, 0, 0.3) );
	transform_laser.setRotation(q);

	while(ros::ok())
	{

		br.sendTransform(tf::StampedTransform(transform_laser, ros::Time::now(), "base_footprint", "base_laser_link"));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
};

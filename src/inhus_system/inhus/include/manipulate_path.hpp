#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include <math.h>

float computePathLength(const nav_msgs::Path* path)
{
	float length=0;

	int path_size = (int)path->poses.size();
	for(int i=0; i<path_size-1; i++)
		length += sqrt( pow(path->poses[i+1].pose.position.x-path->poses[i].pose.position.x,2) + pow(path->poses[i+1].pose.position.y-path->poses[i].pose.position.y,2) );

	return length;
}

int cutPath(const nav_msgs::Path& path1, nav_msgs::Path& path2, const geometry_msgs::Pose2D& pose)
{
	int i_min = -1;

	if((int)path1.poses.size()>0)
	{
		float dist = sqrt(pow(path1.poses[0].pose.position.x-pose.x,2) + pow(path1.poses[0].pose.position.y-pose.y,2));
		float dist_min = dist;
		i_min = 0;
		for(int i=1; i<(int)path1.poses.size(); i++)
		{
			dist = sqrt(pow(path1.poses[i].pose.position.x-pose.x,2) + pow(path1.poses[i].pose.position.y-pose.y,2));
			if(dist < dist_min)
			{
				dist_min = dist;
				i_min = i;
			}
		}

		path2.poses.clear();
		for(int i=i_min; i<(int)path1.poses.size(); i++)
			path2.poses.push_back(path1.poses[i]);
	}

	return i_min;
}

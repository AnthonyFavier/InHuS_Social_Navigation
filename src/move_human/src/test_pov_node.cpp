#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <vector>
#include "move_human/Test.h"
#include "move_human/test_m.h"

using namespace std;

vector<vector<int>> g_map;
vector<vector<int>> g_map_cp;
float resol;

struct Pose
{
	int x;
	int y;
};

bool test_view(geometry_msgs::Pose2D A_real, geometry_msgs::Pose2D B_real)
{
	Pose A_map;
	A_map.x = (int)(A_real.x / resol); A_map.y = (int)(A_real.y / resol);
	Pose B_map;
	B_map.x = (int)(B_real.x / resol); B_map.y = (int)(B_real.y / resol);

	g_map_cp[A_map.y][A_map.x] = 3;
	g_map_cp[B_map.y][B_map.x] = 3;

	cout << "nor A:" << A_real.x << "," << A_real.y << " \tB:" << B_real.x << "," << B_real.y << endl;
	cout << "map A:" << A_map.x << "," << A_map.y << " \tB:" << B_map.x << "," << B_map.y << endl;

	// particular cases
	// if one of the poses is an obstacle
	if(g_map[A_map.y][A_map.x] == 1 || g_map[B_map.y][B_map.x] == 1)
		return false;
	else if(A_map.x == B_map.x || A_map.y == B_map.y) 
	{
		// same place
		if(A_map.x == B_map.x && A_map.y == B_map.y)
			return true;

		// vertical
		else if(A_map.x == B_map.x) 
		{
			for(int i=0; A_map.y + i != B_map.y;)
			{
				int xi = A_map.x;
				int yi = A_map.y + i;

				if(g_map_cp[yi][xi] != 3)
					g_map_cp[yi][xi] = 2;

				if(g_map[yi][xi]==1) // if obstacle
					return false;

				// up
				if(B_map.y > A_map.y)
					i++;
				// down
				else
					i--;
			}
		}

		// horizontal
		else if(A_map.y == B_map.y)
		{
			for(int i=0; A_map.x + i != B_map.x;)
			{
				int xi = A_map.x + i;
				int yi = A_map.y;

				if(g_map_cp[yi][xi] != 3)
					g_map_cp[yi][xi] = 2;

				if(g_map[yi][xi]==1)
					return false;

				// right
				if(B_map.x > A_map.x)
					i++;
				// left
				else
					i--;
			}
		}
	}
	// general cases
	else 
	{
		float m = (float)(B_map.y - A_map.y)/(float)(B_map.x - A_map.x);
		float b = A_map.y - m * A_map.x;

		float marge = 0.9;
		float delta_x = min(marge/abs(m), marge);

		cout << "m = " << m << endl;
		cout << "b = " << b << endl;
		cout << "delta_x = " << delta_x << endl;

		// sign
		if(B_map.x < A_map.x)
			delta_x = -delta_x;

		int i=1;
		bool cond = true;
		while(cond)
		{
			float xi_f = A_map.x + i * delta_x;
			float yi_f = m * xi_f + b;

			int xi = (int)(xi_f);
			int yi = (int)(yi_f);

			cout << "xi_f = " << xi_f << " \tyi_f =" << yi_f << endl;
			cout << "xi = " << xi << " \t\tyi=" << yi << endl;

			if(g_map_cp[yi][xi] != 3)
				g_map_cp[yi][xi] = 2;

			if(g_map[yi][xi]==1) // if obstacle
				return false;

			i++;
			if(delta_x > 0)
				cond = i*delta_x + A_map.x < B_map.x;
			else
				cond = i*delta_x + A_map.x > B_map.x;
		}
	}

	return true;
}

void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	int width = map->info.width;
	int height = map->info.height;
	int cell;

	resol = map->info.resolution;

	for(int i=0; i<height; i++)
	{
		vector<int> line;
		for(int j=0; j<width; j++)
		{
			cell = map->data[width*i+j];
			if(cell == 0)
				line.push_back(0);
			else
				line.push_back(1);
		}
		g_map.push_back(line);
	}

	// SHOW
	for(int y=(int)g_map[0].size()-1; y>=0; y--)
	{
		for(int x=0; x<(int)g_map.size(); x++)
			cout << g_map[y][x] << " ";
		cout << endl;
	}
}

bool test_server(move_human::Test::Request& req, move_human::Test::Response& res)
{	
	cout << "srv A:" << req.A_pose.x << "," << req.A_pose.y << " \tB:" << req.B_pose.x << "," << req.B_pose.y << endl;

	// copy 
	g_map_cp = g_map;

	geometry_msgs::Pose2D A, B;
	A.x = req.A_pose.x; A.y = req.A_pose.y;
	B.x = req.B_pose.x; B.y = req.B_pose.y;

	res.see = test_view(A, B);
	if(res.see)
		cout << "I SEE" << endl;
	else
		cout << "hum ..." << endl;

	// SHOW
	for(int y=(int)g_map_cp[0].size()-1; y>=0; y--)
	{
		for(int x=0; x<(int)g_map_cp.size(); x++)
			cout << g_map_cp[y][x] << " ";
		cout << endl;
	}

	return true;
}

void testCB(const move_human::test_m::ConstPtr& msg)
{
	cout << "sub" << endl;

	// copy 
	g_map_cp = g_map;

	geometry_msgs::Pose2D A, B;
	A.x = msg->poseA.x; A.y = msg->poseA.y;
	B.x = msg->poseB.x; B.y = msg->poseB.y;

	bool see = test_view(A, B);
	if(see)
		cout << "I SEE" << endl;
	else
		cout << "hum ..." << endl;

	// SHOW
	for(int y=(int)g_map_cp[0].size()-1; y>=0; y--)
	{
		for(int x=0; x<(int)g_map_cp.size(); x++)
			cout << g_map_cp[y][x] << " ";
		cout << endl;
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_pov");

	ros::NodeHandle nh;
	ros::Subscriber sub_map_pov = nh.subscribe("/human/pov_map", 1, mapCB);
	ros::Subscriber sub_test = nh.subscribe("/human/test", 1, testCB);
	ros::ServiceServer server = nh.advertiseService("test", test_server);

	ros::spin();

	return 0;
}

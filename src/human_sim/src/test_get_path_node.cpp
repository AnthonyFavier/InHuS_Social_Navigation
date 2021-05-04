#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include <iostream>

#include <boost/thread/thread.hpp>

using namespace std;

float start_x, start_y;
float goal_x, goal_y;

void threadAsk()
{
  float sx, sy, gx, gy;
  sx = start_x;
  sy = start_y;
  gx = goal_x;
  gy = goal_y;

  while(ros::ok())
  {
    //cout << endl << "start x? "; cin >> sx;
    //cout << endl << "start y? "; cin >> sy;
    cout << endl << "goal x? "; cin >> gx;
    cout << endl << "goal y? "; cin >> gy;

    start_x = sx;
    start_y = sy;
    goal_x = gx;
    goal_y = gy;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_path");
  ros::NodeHandle nh;

  start_x = 2.0;
  start_y = 2.0;
  goal_x =  8.0;
  goal_y =  -1.0;

  boost::thread thread_ask(threadAsk);

  ros::ServiceClient client_make_plan = nh.serviceClient<nav_msgs::GetPlan>("move_base/GlobalPlanner/make_plan");

  nav_msgs::GetPlan srv_get_plan;
  srv_get_plan.request.start.header.frame_id = 	"map";
	srv_get_plan.request.goal.header.frame_id = 	"map";
	srv_get_plan.request.tolerance = 		0.1;

  ros::Rate rate(2.0);
  while(ros::ok())
  {
    srv_get_plan.request.start.pose.position.x = 	start_x;
    srv_get_plan.request.start.pose.position.y = 	start_y;
    srv_get_plan.request.goal.pose.position.x = 	goal_x;
    srv_get_plan.request.goal.pose.position.y = 	goal_y;
    if(client_make_plan.call(srv_get_plan))
      cout << "service called" << endl;
    else
      cout << "service failed" << endl;

    ros::spinOnce();
    rate.sleep();

  }

  return 0;
}

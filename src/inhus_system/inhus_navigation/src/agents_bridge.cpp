#include "ros/ros.h"
#include "inhus/PoseVel.h"
#include "cohan_msgs/TrackedAgents.h"
#include "cohan_msgs/TrackedSegmentType.h"
#include "cohan_msgs/AgentType.h"
#include <tf2/LinearMath/Quaternion.h>

ros::Publisher pub_tracked_agents;

void agentCB(const inhus::PoseVel::ConstPtr& msg)
{
    cohan_msgs::TrackedSegment agent_segment;
    agent_segment.type = cohan_msgs::TrackedSegmentType::TORSO;
    geometry_msgs::Pose pose;
    pose.position.x = msg->pose.x;
    pose.position.y = msg->pose.y;
    tf2::Quaternion q;
    q.setRPY(0,0,msg->pose.theta);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    agent_segment.pose.pose = pose;
    agent_segment.twist.twist = msg->vel;
    
    cohan_msgs::TrackedAgent tracked_agent;
    tracked_agent.type = cohan_msgs::AgentType::HUMAN;
    tracked_agent.name = "robot";
    tracked_agent.segments.push_back(agent_segment);
    tracked_agent.track_id = 1;

    cohan_msgs::TrackedAgents tracked_agents;
    tracked_agents.agents.push_back(tracked_agent);
    tracked_agents.header.stamp = ros::Time::now();
    tracked_agents.header.frame_id = "map";

    pub_tracked_agents.publish(tracked_agents);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "input_interface");
	ros::NodeHandle nh;

    ros::Subscriber sub_agent = nh.subscribe("interface/in/robot_pose_vel", 10, agentCB);
    pub_tracked_agents = nh.advertise<cohan_msgs::TrackedAgents>("tracked_agents", 10);

    ros::spin();

	return 0;
}

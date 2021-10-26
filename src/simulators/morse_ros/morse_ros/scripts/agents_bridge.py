#!/usr/bin/python

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import time
from cohan_msgs.msg import TrackedAgents, TrackedAgent, AgentMarkerStamped, TrackedSegment, TrackedSegmentType, AgentType
from geometry_msgs.msg import PointStamped, TwistStamped
import message_filters
# from nav_msgs.msg import Odometry


class MorseAgents(object):

    def __init__(self, num_hum, ns_):
        self.num_hum = num_hum
        self.ns = ns_
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False

    def AgentsPub(self):
        rospy.init_node('Morse_Agents', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        agent_sub = []

        # Subscibe to human agents
        if self.num_hum == 0:
            sig_2 = True

        for agent_id in range(1,self.num_hum+1):
            name = 'human'+str(agent_id)
            if self.ns is not name:
                # agent_sub.append(message_filters.Subscriber("/" + name, AgentMarkerStamped))
                agent_sub.append(message_filters.Subscriber("/pr2_pose_vel", AgentMarkerStamped))

        # Subscribe to the robot
        if False:
            # robot_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.RobotCB)
            robot_sub = rospy.Subscriber("/pr2_pose_vel", AgentMarkerStamped, self.RobotCB)
        else:
            self.sig_2 = True

        self.tracked_agents_pub = rospy.Publisher("tracked_agents", TrackedAgents, queue_size=1)
        pose_msg = message_filters.TimeSynchronizer(agent_sub, 10)
        pose_msg.registerCallback(self.AgentsCB)
        rospy.Timer(rospy.Duration(0.02), self.publishAgents)
        rospy.spin()

    def AgentsCB(self,*msg):
        tracked_agents = TrackedAgents()
        for agent_id in range(1,self.num_hum+1):
            # if self.ns == "human"+str(agent_id):
                # continue
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            agent_segment.pose.pose = msg[agent_id-1].agent.pose
            agent_segment.twist.twist = msg[agent_id-1].agent.velocity
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent_id)
            # tracked_agent.name = "robot"
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
        if(tracked_agents.agents):
            self.agents = tracked_agents
            self.sig_1 = True

    def RobotCB(self, msg):
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.agent.pose
        agent_segment.twist.twist = msg.agent.velocity
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.name = "robot"
        tracked_agent.segments.append(agent_segment)
        self.robot = tracked_agent
        self.sig_2 = True

    def publishAgents(self, event):
        if(self.sig_1):
            self.agents.header.stamp = rospy.Time.now()
            self.agents.header.frame_id = "map"
            # if(self.ns is not ""):
            #     self.agents.agents.append(self.robot)
            for agent_id in range(0, len(self.agents.agents)):
                self.agents.agents[agent_id].track_id = agent_id+1
            self.tracked_agents_pub.publish(self.agents)
            if self.num_hum is not 0:
                self.sig_1 = False
            if self.ns is not "":
                self.sig_2 = False

    # def transform_twist(self, twist, name):
    #     map_twist = TwistStamped()
    #     map_twist.header.frame_id = name+"/odom";
    #
    #     pt = PointStamped()
    #     pt.header.frame_id = name+"/base_footprint";
    #
    #     pt.point.x = twist.twist.linear.x
    #     pt.point.y = twist.twist.linear.y
    #     pt.point.z = twist.twist.linear.z
    #     pt = self.tfBuffer.transform(pt,name+"/odom")
    #     map_twist.twist.linear.x = pt.point.x
    #     map_twist.twist.linear.y = pt.point.y
    #     map_twist.twist.linear.z = pt.point.z
    #
    #     pt.point.x = twist.twist.angular.x
    #     pt.point.y = twist.twist.angular.y
    #     pt.point.z = twist.twist.angular.z
    #     pt = self.tfBuffer.transform(pt,name+"/odom")
    #     map_twist.twist.angular.x = pt.point.x
    #     map_twist.twist.angular.y = pt.point.y
    #     map_twist.twist.angular.z = pt.point.z
    #
    #     return map_twist


if __name__ == '__main__':
    nh = sys.argv[1]
    if(len(sys.argv)<5):
        ns=""
    else:
        ns = sys.argv[2]
    agents = MorseAgents(num_hum=int(nh), ns_=ns)
    agents.AgentsPub()

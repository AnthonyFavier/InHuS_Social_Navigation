#!/usr/bin/python

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and publishes /tracked_humans required for HaNas
# Author: Phani Teja Singamaneni

import sys
import rospy
import time
from human_msgs.msg import TrackedHumans
from human_msgs.msg import TrackedHuman
from human_msgs.msg import TrackedSegmentType
from human_msgs.msg import TrackedSegment
from nav_msgs.msg import Odometry
import message_filters


class StageHumans(object):

    def __init__(self, num_hum):
        self.num_hum = num_hum
        self.tracked_humans_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO

    def HumansPub(self):
        rospy.init_node('Stage_Humans', anonymous=True)
        hum_marker_sub = []
        for human_id in range(1,self.num_hum+1):
            name = 'human'+str(human_id)
            hum_marker_sub.append(message_filters.Subscriber("/" + name + "/base_pose_ground_truth", Odometry))
        self.tracked_humans_pub = rospy.Publisher("/tracked_humans", TrackedHumans, queue_size=1)
        pose_msg = message_filters.TimeSynchronizer(hum_marker_sub, 10)
        pose_msg.registerCallback(self.HumansCB)
        rospy.spin()

    def HumansCB(self,*msg):
        tracked_humans = TrackedHumans()
        for human_id in range(1,self.num_hum+1):
            human_segment = TrackedSegment()
            human_segment.type = self.Segment_Type
            human_segment.pose.pose = msg[human_id-1].pose.pose
            human_segment.twist.twist = msg[human_id-1].twist.twist
            tracked_human = TrackedHuman()
            tracked_human.track_id = human_id
            tracked_human.segments.append(human_segment)
            tracked_humans.humans.append(tracked_human)
        if(tracked_humans.humans):
            tracked_humans.header.stamp = rospy.Time.now()
            tracked_humans.header.frame_id = 'map'
            self.tracked_humans_pub.publish(tracked_humans)


if __name__ == '__main__':
    nh = sys.argv[1]
    humans = StageHumans(num_hum=int(nh))
    humans.HumansPub()

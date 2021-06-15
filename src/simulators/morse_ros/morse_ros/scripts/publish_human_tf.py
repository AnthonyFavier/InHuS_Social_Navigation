#!/usr/bin/env python
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

class BroadCastTF(object):
    def __init__(self):
        rospy.init_node('h_tf_broadcast')
        self.name = rospy.get_param('~ns')
        self.br = tf.TransformBroadcaster()
        self.a_pose = Odometry()
        rospy.Subscriber('/%s/base_pose_ground_truth' % self.name, Odometry, self.tfCB)
        # self.agent_base_pub_ = rospy.Publisher('/%s/base_pose_ground_truth' % self.name, Odometry, queue_size=1)
        rospy.Timer(rospy.Duration(1./100),self.tfPub)
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()

    def tfCB(self, msg):
        self.a_pose = msg

    def tfPub(self, event):
        if(self.a_pose):
            now = rospy.Time.now()
            self.br.sendTransform((self.a_pose.pose.pose.position.x, self.a_pose.pose.pose.position.y, 0),
                         (self.a_pose.pose.pose.orientation.x,self.a_pose.pose.pose.orientation.y, self.a_pose.pose.pose.orientation.z,self.a_pose.pose.pose.orientation.w),
                         now,
                         '%s/base_footprint' % self.name,
                         '%s/odom' % self.name)

            self.br.sendTransform((0, 0 ,0),
                         (0,0,0,1),
                         now,
                         '%s/base_link' % self.name,
                         '%s/base_footprint' % self.name)

            self.br.sendTransform((0.275, 0 ,0.05),
                         (0,0,0,1),
                         now,
                         '%s/base_laser_link' % self.name,
                         '%s/base_link' % self.name)

            # base_truth =  Odometry()
            # base_truth.header.stamp = now
            # base_truth.pose.pose = self.a_pose.agent.pose
            # base_truth.twist.twist = self.a_pose.agent.velocity
            # self.agent_base_pub_.publish(base_truth)

if __name__ == '__main__':
    agent_tf = BroadCastTF()

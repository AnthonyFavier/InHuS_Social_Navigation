#!/usr/bin/python

# Brief: This function publishes /joint_states for tucked_arm_pose of PR2, necessary for displaying robot robot_description in Rviz
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('stage_joints', anonymous=True)
    joint_states = JointState()
    joint_states.header.frame_id = "/joint_states"
    joint_states.name = ["laser_tilt_mount_joint","fl_caster_rotation_joint","fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_rotation_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_rotation_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_rotation_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint","r_gripper_motor_slider_joint","r_gripper_motor_screw_joint","r_gripper_l_finger_joint","r_gripper_r_finger_joint","r_gripper_l_finger_tip_joint","r_gripper_r_finger_tip_joint","r_gripper_joint","l_gripper_motor_slider_joint","l_gripper_motor_screw_joint","l_gripper_l_finger_joint","l_gripper_r_finger_joint","l_gripper_l_finger_tip_joint","l_gripper_r_finger_tip_joint","l_gripper_joint","torso_lift_joint","torso_lift_motor_screw_joint","head_pan_joint","head_tilt_joint","l_shoulder_pan_joint","l_shoulder_lift_joint","l_upper_arm_roll_joint","l_elbow_flex_joint","l_forearm_roll_joint","l_wrist_flex_joint","l_wrist_roll_joint","r_shoulder_pan_joint","r_shoulder_lift_joint","r_upper_arm_roll_joint","r_elbow_flex_joint","r_forearm_roll_joint","r_wrist_flex_joint","r_wrist_roll_joint"]
    joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -2.9802322387695312e-08, 0.06914562731981277, 1.2399593591690063, 1.7890771627426147, -1.6932392120361328, -1.7343393564224243, -0.09060896933078766, -0.07657696306705475, -0.0138227678835392, 1.097334384918213, -1.5566887855529785, -2.114457845687866, -1.4083462953567505, -1.8511372804641724, 0.2240774929523468]
    r = rospy.Rate(50)
    joint_states_pub = rospy.Publisher("/stage_joint_states", JointState, queue_size=1)
    while not rospy.is_shutdown():
        joint_states.header.stamp = rospy.Time.now()
        joint_states_pub.publish(joint_states)
        r.sleep()

if __name__ == "__main__":
    main()

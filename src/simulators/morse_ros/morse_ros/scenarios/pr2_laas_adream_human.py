#!/usr/bin/python
# Author: Phani Teja Singamaneni

import sys
import rospy
from morse.builder import *
from morse.core.services import service

num_humans = 1
locations = [[3.0, 1.0, 0.0],[7.0, 5.0, 0.0],[7.0, 14.0, 0.0]]
orientations = [0.0,0.7,1.57]

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")

def add_human(h_id):

    if(h_id==1):
        human = Human()
    else:
        human = Human(filename="human_rig"+str(h_id))
    human.properties(WorldCamera = True)
    human.properties(GroundRobot = True)

    name = "human" + str(h_id)

    human_pose = Pose()
    human.append(human_pose)
    human_pose.add_interface("ros", topic="/morse/"+name+"_pose")

    human_velocity = Velocity()
    human.append(human_velocity)
    human_velocity.add_interface("ros", topic="/morse/"+name+"_vel")

    human_motion = MotionXYW()
    human_motion.properties(ControlType='Position')
    human.append(human_motion)
    human_motion.add_interface("ros", topic="/morse/" + name + "_cmd_vel")

    human.append(clock)
    return human


# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
# for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
# pr2 = NavPR2(with_keyboard=True, show_laser=True, laser_z=0.05)
pr2 = NavPR2(with_keyboard=True)
pr2.add_interface("ros")
keyboard = Keyboard()
pr2.append(keyboard)
robot_velocity = Velocity()
pr2.append(robot_velocity)
robot_velocity.add_interface("ros", topic="/morse/robot_vel")
robot_pose = Pose()
pr2.append(robot_pose)
robot_pose.add_interface("ros", topic="/morse/robot_pose")

# For fake localization
ground_truth = Odometry()
pr2.append(ground_truth)
ground_truth.add_interface("ros", topic="base_pose_ground_truth")

# put the robot and humans in some good places and add clock
pr2.translate(2.0, 2.0, 0.0)
pr2.append(clock)

# HumanArray humans
humans = []
for h_id in range(0,num_humans):
    humans.append(add_human(h_id+1))

# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

#Place Humans at different locations and attach cameras
for h_id in range(0,num_humans):
    humans[h_id].translate(locations[h_id][0],locations[h_id][1],locations[h_id][2])
    humans[h_id].rotate(z=orientations[h_id])
    humans[h_id].append(clock)


env.use_relative_time(True)
env.create()

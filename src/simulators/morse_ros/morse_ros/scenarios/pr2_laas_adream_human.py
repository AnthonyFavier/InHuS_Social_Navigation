#!/usr/bin/python
# Author: Phani Teja Singamaneni

import sys
import rospy
from morse.builder import *
from morse.core.services import service

namespace = "morse_agents/"

num_humans = 1
locations = [[1.3, 3.0, 0.0],[4.0, -2.0, 0.0],[4.4, 11.6, 0.0]]
orientations = [1.57,0.7,1.57]

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

    ## human_marker sensor for the human
    human_marker = AgentMarker()
    human.append(human_marker)
    human_marker.add_interface("ros", topic=namespace+name+"/marker")

    human_motion = MotionXYW()
    human_motion.properties(ControlType='Position')

    human_odom = Odometry()
    human_odom.add_interface("ros",topic=namespace+name+"/odom",
                             frame_id = name+"/odom",
                             child_frame_id = name+"/base_footprint")
    human_odom.add_interface("ros",topic= namespace+name+"/base_pose_ground_truth",
                             frame_id = name+"/odom",
                             child_frame_id = name+"/base_footprint")
    human.append(human_odom)

    human.append(human_motion)
    human_motion.add_interface("ros", topic= namespace+name+"/cmd_vel")
    human.append(clock)
    return human


# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
# for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
# pr2 = NavPR2(with_keyboard=True, show_laser=True, laser_z=0.05)
robot_name = "pr2"
pr2 = NavPR2(with_keyboard=True, show_laser=False, laser_z=0.05, namespace=namespace+robot_name+"/")
pr2.add_interface("ros")

# For fake localization
ground_truth = Odometry()
pr2.append(ground_truth)
ground_truth.add_interface("ros", topic= namespace+robot_name+"/base_pose_ground_truth")

# Agent Marker to get the absolute position and velocity
robot_marker = AgentMarker()
robot_marker.add_interface("ros", topic= namespace+robot_name+"/marker")
pr2.append(robot_marker)

# put the robot and humans in some good places and add clock
pr2.translate(6.4, 6.4, 0.0)
pr2.rotate(z=-1.57)
pr2.append(clock)

# HumanArray humans
humans = []
for h_id in range(0,num_humans):
    humans.append(add_human(h_id+1))


# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

cameras = []

#Place Humans at different locations and attach cameras
for h_id in range(0,num_humans):
    humans[h_id].translate(locations[h_id][0],locations[h_id][1],locations[h_id][2])
    humans[h_id].rotate(z=orientations[h_id])
    humans[h_id].append(clock)
    # human_camera = VideoCamera("FPV")
    # human_camera.translate(0.10, 0, 1.63)
    # human_camera.rotate(0, -0.17, 0)
    # human_camera.properties(cam_width=1920, cam_height=1080)
    # cameras.append(human_camera)
    # humans[h_id].append(cameras[h_id])

# env.select_display_camera(cameras[0])
env.use_relative_time(True)
env.create()

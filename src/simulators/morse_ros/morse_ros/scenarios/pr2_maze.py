# Author: Phani Teja Singamaneni

import sys
from morse.builder import *

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")

# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
# for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
pr2 = NavPR2(with_keyboard=True, show_laser=False, laser_z=0.05)
pr2.add_interface("ros")
pr2.append(clock)

# For fake localization
ground_truth = Odometry()
pr2.append(ground_truth)
ground_truth.add_interface("ros", topic="base_pose_ground_truth")

# set the environment to laas_adream
env = Environment('maze.blend', fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# put the robot in some good place
pr2.translate(2.0, 2.0, 0.0)

env.use_relative_time(True)
env.create()

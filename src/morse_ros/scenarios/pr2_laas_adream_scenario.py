import sys
from morse.builder import *

# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
#  for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
pr2 = NavPR2()
pr2.add_interface("ros")

# set the environment to laas_adream
env = Environment('laas_adream.blend', fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
pr2.append(clock)

# create teleport actuator
#teleport = Teleport()
#teleport.add_interface("ros", topic="pr2_teleport_pose")
#pr2.append(teleport)

# rear laser for pr2 if asked
if '--rear_laser' in sys.argv:
    rear_laser = Hokuyo()
    rear_laser.translate(x=-0.275, z=0.252)
    rear_laser.rotate(z=3.14)
    rear_laser.properties(laser_range = 30.0)
    rear_laser.properties(resolution = 1.0)
    rear_laser.properties(scan_window = 180.0)
    rear_laser.create_laser_arc()
    rear_laser.add_stream("ros", topic="rear_scan", frame_id="rear_laser_link")
    pr2.append(rear_laser)

# put the robot in some good place
pr2.translate(2.0, 2.0, 0.0)

env.use_relative_time(True)
env.create()

import sys
import rospy
from morse.builder import *
from morse.core.services import service

num_humans = 1
locations = [[3.0, 1.0, 0.0],[7.0, 5.0, 0.0],[7.0, 14.0, 0.0]]
orientations = [0.0,0.7,1.57]

def add_human(h_id):

    if(h_id==1):
        human = Human()
    else:
        human = Human(filename="human_rig"+str(h_id))
    human.properties(WorldCamera = True)

    name = "human" + str(h_id)

    # human_marker sensor for the human
    human_marker = HumanMarker()

    human.append(human_marker)
    human_marker.add_interface("ros", topic="/"+name)

    human_motion = MotionXYW()
    human_motion.properties(ControlType='Position')

    human.append(human_motion)
    human_motion.add_interface("ros", topic="/" + name + "/cmd_vel")
    return human


# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
#  for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
pr2 = NavPR2(with_keyboard=False)
pr2.add_interface("ros")
keyboard = Keyboard()
pr2.append(keyboard)

# teleport actuator for the pr2
#teleport_pr2 = Teleport()
#pr2.append(teleport_pr2)
#teleport_pr2.add_interface("ros", topic="pr2_teleport_pose")

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


# HumanArray humans
humans = []
for h_id in range(0,num_humans):
    humans.append(add_human(h_id+1))

# set the environment to laas_adream
#env = Environment("laas_adream.blend", fastmode=False)
env = Environment("new_map_modified.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")

# put the robot and humans in some good places and add clock
pr2.translate(2.0, 2.0, 0.0)
pr2.append(clock)

cameras = []

#Place Humans at different locations and append cameras
for h_id in range(0,num_humans):
    humans[h_id].translate(locations[h_id][0],locations[h_id][1],locations[h_id][2])
    humans[h_id].rotate(z=orientations[h_id])
    humans[h_id].append(clock)
    human_camera = VideoCamera("FPV")
    human_camera.translate(0.10, 0, 1.63)
    human_camera.rotate(0, -0.17, 0)
    human_camera.properties(cam_width=1920, cam_height=1080)
    cameras.append(human_camera)
    humans[h_id].append(cameras[h_id])

env.select_display_camera(cameras[0])
env.use_relative_time(True)
env.create()

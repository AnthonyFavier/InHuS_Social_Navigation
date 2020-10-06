import sys
from morse.builder import *

# pr2 robot with laser (scan) and odometry (odom) sensors, and actuators
#  for armature (joint_trajectory_contorller) and wheels (cmd_vel) to the scene
pr2 = NavPR2(with_keyboard=False)
pr2.add_interface("ros")

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

# adding human model
human = Human()
human.properties(WorldCamera = True)

# pose sensor for the human
pose = Pose()
human.append(pose)
pose.add_interface("ros", topic="human_pose")
human_motion = MotionXYW()
human.append(human_motion)
human_motion.properties(ControlType='Position')
human_motion.add_interface("ros", topic="human_cmd_vel")
human_camera = VideoCamera("FPV")
human_camera.translate(0.20, 0, 1.63)
human_camera.rotate(0, -0.17, 0)
human.append(human_camera)
human_camera.properties(cam_width=800, cam_height=600)

keyboard = Keyboard()
human.append(keyboard)

# teleport actuator for the human
#teleport_human = Teleport()
#human.append(teleport_human)
#teleport_human.add_interface("ros", topic="human_teleport_pose")

# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# put the robot and human in some good places
pr2.translate(2.0, 2.0, 0.0)
human.translate(3.0, 3.0, 0.0)

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
pr2.append(clock)
human.append(clock)

env.use_relative_time(True)
env.create()

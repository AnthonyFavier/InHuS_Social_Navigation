#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <nev_tutorial> environment

Feel free to edit this template as you like!
"""

from morse.builder import *

human = Human()
human.properties(GoundRobot = True)
human.add_interface('ros')

motion = MotionXYW()
motion.properties(ControlType = 'Position')
human.append(motion)
motion.add_interface('ros', topic="/cmd_vel")

scan = Hokuyo()
scan.translate(x=0, z=0.3)
scan.add_interface('ros')
human.append(scan)
scan.properties(Visible_arc = True)
scan.properties(laser_range = 30.0)
scan.properties(resolution = 1)
scan.properties(scan_window = 180.0)
scan.create_laser_arc()

keyboard = Keyboard()
#robot = Morsy()
#robot.translate(-2,2,0)
human.append(keyboard)

odometry = Odometry()
human.append(odometry)
odometry.add_interface('ros', topic="/human/odom")

pose = Pose();
human.append(pose)
pose.add_interface('ros', topic="/human/pose")

env = Environment('sandbox', fastmode = False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])


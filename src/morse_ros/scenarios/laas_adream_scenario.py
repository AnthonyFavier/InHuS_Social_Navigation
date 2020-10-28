from morse.builder import *

# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([0.9, 0.0 , 1.57])

# creates a new instance of the morsy robot
human = Human()
human.properties(GroundRobot = True)
#human.translate(x=2, y=2, z=0); #start
human.translate(x=10.1, y=2.6, z=0); #corridor
human.rotate(0,0,1.57)

motionH = MotionXYW()
motionH.properties(ControlType = 'Position')
human.append(motionH)
motionH.add_interface('ros', topic="human_cmd_vel")

pose = Pose()
human.append(pose)
pose.add_interface('ros', topic="morse/human_pose")

################################################

pr2 = BarePR2()
pr2.properties(GroundRobot = True)
#pr2.translate(5.0, 2.0, 0.0) #start
#pr2.translate(5.8, 4.6, 0.0) #block appart
pr2.translate(10, 13.5, 0.0) #corridor
pr2.rotate(0,0,-1.57)

keyboard = Keyboard()
pr2.append(keyboard)

motionR = MotionXYW()
motionR.properties(ControlType = 'Position')
pr2.append(motionR)
motionR.add_interface('ros', topic="robot_cmd_vel")

pose_pr2 = Pose()
pr2.append(pose_pr2)
pose_pr2.add_interface('ros', topic="morse/robot_pose")

# add clock
clock = Clock()
clock.add_interface('ros', topic="clock")
human.append(clock)

env.use_relative_time(True)
env.create()

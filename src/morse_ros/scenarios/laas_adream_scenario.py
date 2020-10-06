from morse.builder import *

# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# creates a new instance of the morsy robot
human = Human()
human.properties(GroundRobot = True)
human.translate(x=2, y=2, z=0);

motion = MotionXYW()
motion.properties(ControlType = 'Position')
human.append(motion)
motion.add_interface('ros', topic="/cmd_vel")

keyboard = Keyboard()
human.append(keyboard)

pose = Pose()
human.append(pose)
pose.add_interface('ros', topic="morse/human_pose")

odometry = Odometry()
human.append(odometry)
odometry.add_interface('ros', topic="morse/human_odom")

# add clock
clock = Clock()
clock.add_interface('ros', topic="clock")
human.append(clock)

env.use_relative_time(True)
env.create()

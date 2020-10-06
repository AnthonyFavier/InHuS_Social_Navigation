from morse.builder import *

# fakerobot
fakerobot = FakeRobot()
# fakerobot.add_interface("ros")

# set the environment to laas_adream
env = Environment('laas_adream.blend', fastmode=True)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
fakerobot.append(clock)

# create teleport actuator
teleport = Teleport()
teleport.add_interface("ros", topic="fakerobot_teleport_pose")
fakerobot.append(teleport)

# motion controller
motion = MotionXYW()
motion.properties(ControlType = 'Position')
motion.add_stream("ros", topic="cmd_vel")
fakerobot.append(motion)

# laser scanner
hokuyo = Hokuyo()
hokuyo.translate(z=0.25)
hokuyo.add_stream("ros", topic="base_scan")
fakerobot.append(hokuyo)

# odometry
odometry = Odometry()
odometry.add_stream("ros", topic="odom")
fakerobot.append(odometry)

# put the robot in some good place
fakerobot.translate(2.0, 2.0, 0.0)

env.use_relative_time(True)
env.create()

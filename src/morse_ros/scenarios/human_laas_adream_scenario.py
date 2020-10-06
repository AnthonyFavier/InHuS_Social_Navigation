from morse.builder import *

# adding human model
human = Human()
human.properties(WorldCamera = True)

# pose sensor for the human
pose = Pose()
human.append(pose)
pose.add_interface("ros", topic="human_pose")

# new instance of the  teleport actuator for the human
teleport = Teleport()
human.append(teleport)
teleport.add_interface("ros", topic="human_teleport_pose")

# set the environment to laas_adream
env = Environment("laas_adream.blend", fastmode=True)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# put the human in better places
human.translate(2.0, 2.0, 0.0)

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
human.append(clock)

env.use_relative_time(True)
env.create()

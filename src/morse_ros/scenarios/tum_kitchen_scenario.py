from morse.builder import *

# set the environment to tum_kitchen
env = Environment("tum_kitchen.blend", fastmode=True)
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.0470, 0, 0.7854])

# creates a new instance of the morsy robot
morsy = Morsy()

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
morsy.append(clock)

env.use_relative_time(True)
env.create()

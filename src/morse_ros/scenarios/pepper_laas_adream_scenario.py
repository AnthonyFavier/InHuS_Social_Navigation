from morse.builder import *

class Pepper(Robot):
    def __init__(self, name = None):
        Robot.__init__(self, "pepper_low_poly.blend", name)

pepper = Pepper()

env = Environment("laas_adream_free_living_room.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

# put the robot and human in some good places
pepper.translate(2.0, 2.0, 0.0)

odom = Odometry()
odom.add_interface("ros", topic="/odom_diffdrive", child_frame_id="/base_link")
pepper.append(odom)

motion = MotionXYW()
motion.add_interface("ros", topic="/cmd_vel")
pepper.append(motion)

#scan = Hokuyo()
#scan.translate(x=0.32, z=0.252)
#pepper.append(scan)
#scan.properties(Visible_arc = False)
#scan.properties(laser_range = 30.0)
#scan.properties(resolution = 1.0)
#scan.properties(scan_window = 180.0)
#scan.create_laser_arc()
#scan.add_interface('ros', topic='/base_scan')

kb = Keyboard()
pepper.append(kb)

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
pepper.append(clock)

env.use_relative_time(True)
env.create()

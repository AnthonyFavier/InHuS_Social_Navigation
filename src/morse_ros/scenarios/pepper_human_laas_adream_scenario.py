from morse.builder import *

class Pepper(Robot):
    def __init__(self, name = None):
        Robot.__init__(self, "pepper_low_poly.blend", name)

pepper = Pepper()

env = Environment("laas_adream_free_living_room.blend", fastmode=False)
env.set_camera_location([18.0, 4.0, 10.0])
env.set_camera_rotation([1.0, 0.0 , 1.57])

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

# add clock
clock = Clock()
clock.add_interface("ros", topic="clock")
pepper.append(clock)
human.append(clock)

# put the robot and human in some good places
pepper.translate(2.0, 2.0, 0.0)
human.translate(3.0, 3.0, 0.0)

env.use_relative_time(True)
env.create()

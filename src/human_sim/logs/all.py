import sys, os

if len(sys.argv)>=3:
    os.system("python path_color.py poseLog.txt B " + sys.argv[1] + " " + sys.argv[2])
    os.system("python graph.py log.txt " + sys.argv[1] + " " + sys.argv[2])
elif len(sys.argv)>=2:
    os.system("python path_color.py poseLog.txt B " + sys.argv[1])
    os.system("python graph.py log.txt " + sys.argv[1])
else:
    os.system("python path_color.py poseLog.txt B")
    os.system("python graph.py log.txt")

    

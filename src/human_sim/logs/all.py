import sys, os

if len(sys.argv)>=3:
    os.system("python colored_path.py log_data/poseLog.txt B " + sys.argv[1] + " " + sys.argv[2])
    os.system("python exec_graphs.py log_data/log.txt " + sys.argv[1] + " " + sys.argv[2])
elif len(sys.argv)>=2:
    os.system("python colored_path.py log_data/poseLog.txt B " + sys.argv[1])
    os.system("python exec_graphs.py log_data/log.txt " + sys.argv[1])
else:
    os.system("python colored_path.py log_data/poseLog.txt B")
    os.system("python exec_graphs.py log_data/log.txt")

import sys, os

if len(sys.argv)>=3:
    os.system("python colored_path.py inhus_logs/poseLog.txt B " + sys.argv[1] + " " + sys.argv[2])
    os.system("python exec_graphs.py inhus_logs/log.txt " + sys.argv[1] + " " + sys.argv[2])
elif len(sys.argv)>=2:
    os.system("python colored_path.py inhus_logs/poseLog.txt B " + sys.argv[1])
    os.system("python exec_graphs.py inhus_logs/log.txt " + sys.argv[1])
else:
    os.system("python colored_path.py inhus_logs/poseLog.txt B")
    os.system("python exec_graphs.py inhus_logs/log.txt")

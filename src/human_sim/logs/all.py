import sys, os, threading

def graph_thread():

    if len(sys.argv)>=3:
        os.system("python graph.py log.txt " + sys.argv[1] + " " + sys.argv[2])
    elif len(sys.argv)>=2:
        os.system("python graph.py log.txt " + sys.argv[1])
    else:
        os.system("python graph.py log.txt")

def path_color_thread():

    if len(sys.argv)>=3:
        os.system("python path_color.py poseLog.txt B " + sys.argv[1] + " " + sys.argv[2])
    elif len(sys.argv)>=2:
        os.system("python path_color.py poseLog.txt B " + sys.argv[1])
    else:
        os.system("python path_color.py poseLog.txt B")

if __name__ == "__main__":

    x = threading.Thread(target=graph_thread)
    x.start()
    path_color_thread()
    

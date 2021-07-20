################################
# Explanation to show log data #
################################

$ python all.py [start] [end]
Shows both the time colored paths of agents and the graphs of the different metrics.
The arguments start and end are optional and allows to show only the results after 'start' and before 'end' if specified.
This script calls the two following scripts colored_path.py and exec_graphs.py.

$ python colored_path.py [log_file_path] [which] [start] [end]
Shows the timed colored paths.
The two first arguments are mandatory. The first one the name of the position log file (default = "poseLog.txt"). And the second one specifies if we want to print both paths (value='B', selected by default in all.py), only the human (value='H') or only the robot (value='R').
The two next arguments 'start' and 'end' are the same as above and still optional.

$ python exec_graphs.py [log_file_path] [start] [end]
Shows the metrics recorded or computed by InHuS.
The first argument is the log file (default = "log.txt") and is mandatory.
The two next ones are again to limit the shown temporal window.

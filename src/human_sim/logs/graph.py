import matplotlib.pyplot as plt
import sys

if len(sys.argv)>=2:
    filename =sys.argv[1]
f = open(filename, "r")

# start
string = f.readline()

list_path_data = []
list_path_time = []

list_dist_data = []
list_dist_time = []

list_first_data = []
list_first_time = []

list_state_exec_data = []
list_state_exec_time = []

list_state_approach_data = []
list_state_approach_time = []

list_state_blocked_data = []
list_state_blocked_time = []

list_ttc_data = []
list_ttc_time = []

value_state = -1

for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")

        if mylist[2] == 'SUPERVISOR' or mylist[2] == 'HUMAN_MODEL':
            if mylist[3] == 'DIST':
                list_dist_data.append(float(mylist[4]))
                list_dist_time.append(float(mylist[5]))

            elif mylist[3] == 'FIRST':
                list_first_data.append(float(mylist[5]))
                list_first_time.append(float(mylist[4]))

            elif mylist[3] == 'STATE':
                if mylist[4] == 'PROGRESS':
                    list_state_exec_data.append(value_state)
                    list_state_exec_time.append(float(mylist[5]))

                elif mylist[4] == 'APPROACH':
                    list_state_approach_data.append(value_state)
                    list_state_approach_time.append(float(mylist[5]))

                elif mylist[4] == 'BLOCKED':
                    list_state_blocked_data.append(value_state)
                    list_state_blocked_time.append(float(mylist[5]))

            elif mylist[3] == 'TTC':
                list_ttc_data.append(float(mylist[4]))
                list_ttc_time.append(float(mylist[5]))

            else:
                time = float(mylist[3])
                path = float(mylist[4])

                list_path_time.append(time)
                list_path_data.append(path)

f.close()

fig, (ax1, ax3) = plt.subplots(2, 1)

ax1.set_xlabel('time (s)')
ax1.set_ylabel('path length (m)')
ax1.plot(list_path_time, list_path_data, 'b+')
ax1.plot(list_first_time, list_first_data, 'ro')
ax1.plot(list_state_exec_time, list_state_exec_data, 'gs')
ax1.plot(list_state_approach_time, list_state_approach_data, 'ys')
ax1.plot(list_state_blocked_time, list_state_blocked_data, 'rs')

color = 'tab:grey'
ax2 = ax1.twinx()
ax2.set_ylabel('human-robot distance (m)', color=color)
ax2.plot(list_dist_time, list_dist_data, '--', color=color)
ax2.set_ylim(bottom=0)

ax3.set_xlabel('time (s)')
ax3.set_ylabel('TTC (s)')
ax3.plot(list_ttc_time, list_ttc_data, 'k+')
ax3.set_ylim(bottom=0)
ax3.set_xlim(left=ax1.get_xbound()[0], right=ax1.get_xbound()[1])

if len(sys.argv)>=4:
    left_lim = int(sys.argv[2])
    right_lim = int(sys.argv[3])
    ax1.set_xlim(left=left_lim, right=right_lim)
    ax3.set_xlim(left=left_lim, right=right_lim)

fig.tight_layout()

plt.show()

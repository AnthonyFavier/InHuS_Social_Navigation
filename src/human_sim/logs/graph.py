import matplotlib.pyplot as plt
import sys

for arg in sys.argv:
    filename = arg

f = open(filename, "r")

# start
string = f.readline()

list_time = []
list_path = []

list_dist_time = []
list_dist_dist = []

list_first_time = []
list_first_path = []

list_state_exec_d = []
list_state_exec_time = []

list_state_blocked_d = []
list_state_blocked_time = []

value_state = -10

for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")

        if mylist[3] == 'DIST':
            dist = float(mylist[4])
            time = float(mylist[5])

            list_dist_time.append(time)
            list_dist_dist.append(dist)

        elif mylist[3] == 'FIRST':
            path = float(mylist[5])
            time = float(mylist[4])

            list_first_path.append(path)
            list_first_time.append(time)

        elif mylist[3] == 'STATE':
            if mylist[4] == 'EXEC':
                list_state_exec_d.append(value_state)
                list_state_exec_time.append(float(mylist[5]))

            elif mylist[4] == 'BLOCKED':
                list_state_blocked_d.append(value_state)
                list_state_blocked_time.append(float(mylist[5]))

        else:
            time = float(mylist[3])
            path = float(mylist[4])

            list_time.append(time)
            list_path.append(path)

f.close()

fig, ax1 = plt.subplots()
ax1.set_xlabel('time (s)')
ax1.set_ylabel('path length')
ax1.plot(list_time, list_path, 'b+')
ax1.plot(list_first_time, list_first_path, 'ro')

ax1.plot(list_state_exec_time, list_state_exec_d, 'gs')
ax1.plot(list_state_blocked_time, list_state_blocked_d, 'rs')

color = 'tab:grey'
ax2 = ax1.twinx()
ax2.set_ylabel('human-robot distance', color=color)
ax2.plot(list_dist_time, list_dist_dist, '--', color=color)
ax2.set_ylim(bottom=0)

fig.tight_layout()

#plt.plot(list_time, list_path, 'b+')
#plt.plot(list_first_time, list_first_path, 'ro')
#plt.plot(list_dist_time, list_dist_dist, ',')

plt.show()

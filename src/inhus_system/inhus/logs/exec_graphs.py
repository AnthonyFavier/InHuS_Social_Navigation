import matplotlib.pyplot as plt
import sys

if len(sys.argv)>=2:
    filename =sys.argv[1]
f = open(filename, "r")

# start
string = f.readline()

path_data = []
path_time = []

dist_data = []
dist_time = []

vel_h_data = []
vel_h_time = []

vel_r_data = []
vel_r_time = []

first_data = []
first_time = []

state_exec_data = []
state_exec_time = []

state_approach_data = []
state_approach_time = []

state_blocked_data = []
state_blocked_time = []

ttc_data = []
ttc_time = []

rel_spd_data = []
rel_spd_time = []

surprise_data = []
surprise_time = []

seen_ratio_data = []
seen_ratio_time = []

value_state = -1

# Fill data
for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")

        if mylist[2] == 'SUPERVISOR' or mylist[2] == 'HUMAN_MODEL' or mylist[2] == 'LOG' or mylist[2] == 'CONFLICT_MANAGER':
            if mylist[3] == 'DIST':
                dist_data.append(float(mylist[4]))
                dist_time.append(float(mylist[5]))

            elif mylist[3] == 'FIRST':
                first_data.append(float(mylist[4]))
                first_time.append(float(mylist[5]))

            elif mylist[3] == "PATH":
                path_data.append(float(mylist[4]))
                path_time.append(float(mylist[5]))

            elif mylist[3] == 'STATE':
                if mylist[4] == 'PROGRESS':
                    state_exec_data.append(value_state)
                    state_exec_time.append(float(mylist[5]))

                elif mylist[4] == 'APPROACH':
                    state_approach_data.append(value_state)
                    state_approach_time.append(float(mylist[5]))

                elif mylist[4] == 'BLOCKED':
                    state_blocked_data.append(value_state)
                    state_blocked_time.append(float(mylist[5]))

            elif mylist[3] == 'TTC':
                ttc_data.append(float(mylist[4]))
                ttc_time.append(float(mylist[5]))

            elif mylist[3] == 'VEL_H':
                vel_h_data.append(float(mylist[4]))
                vel_h_time.append(float(mylist[0]))

            elif mylist[3] == 'VEL_R':
                vel_r_data.append(float(mylist[4]))
                vel_r_time.append(float(mylist[0]))

            elif mylist[3] == 'REL_SPD':
                rel_spd_data.append(float(mylist[4]))
                rel_spd_time.append(float(mylist[5]))

            elif mylist[3] == 'SEEN_RATIO':
                seen_ratio_data.append(float(mylist[4]))
                seen_ratio_time.append(float(mylist[5]))

            elif mylist[3] == 'SURPRISED':
                surprise_data.append(0)
                surprise_time.append(float(mylist[4]))
f.close()

# Treat data

for i, surp_time in enumerate(surprise_time):
    diff_min = -1
    j_min=0
    for j, time in enumerate(seen_ratio_time):
        diff = abs(time - surp_time)
        if diff_min == -1:
            diff_min = diff
        else:
            if diff <= diff_min:
                diff_min = diff
                j_min = j
            else:
                break
    surprise_data[i] = seen_ratio_data[j_min]

min_x = min(dist_time[0], vel_h_time[0], vel_r_time[0], rel_spd_time[0], seen_ratio_time[0])
max_x = max(dist_time[-1], vel_h_time[-1], vel_r_time[-1], rel_spd_time[-1], seen_ratio_time[-1])

fig, (ax1, ax2, ax5, ax7) = plt.subplots(4, 1)
plt.subplots_adjust(left=0.10, right=0.91, bottom=0.10, top=0.98, hspace = 0.24)

# Plots

# PATHS #
#ax1.set_xlabel('time (s)')
ax1.set_ylabel('path length (m)')
ax1.plot(path_time, path_data, 'b+')
ax1.plot(first_time, first_data, 'ro')
ax1.plot(state_exec_time, state_exec_data, 'gs')
ax1.plot(state_approach_time, state_approach_data, 'ys')
ax1.plot(state_blocked_time, state_blocked_data, 'rs')
ax1.set_xlim(left=min_x, right=max_x)

# DIST + VELS #
color = 'tab:grey'
#ax2.set_xlabel('time (s)')
ax2.set_ylabel('human-robot \n distance (m)')
ax2.plot(dist_time, dist_data, '--', color=color)
ax2.set_xlim(left=min_x, right=max_x)
ax2.set_ylim(bottom=0)

ax3 = ax2.twinx()
ax3.set_ylabel('speeds (m/s)')
#ax3.set_ylabel('vel_h (m/s)', color=color)
vel_h, = ax3.plot(vel_h_time, vel_h_data, '-b')
ax3.set_ylim(bottom=0)

ax4 = ax2.twinx()
#ax4.set_ylabel('vel_r (m/s)', color=color)
vel_r, = ax4.plot(vel_r_time, vel_r_data, '-r')
ax4.set_ylim(bottom=0)

maxi_vel = max(ax3.get_ybound()[1], ax4.get_ybound()[1])
maxi_vel = 1.5
ax3.set_ylim(top=maxi_vel)
ax4.set_ylim(top=maxi_vel)

ax2.legend([vel_h, vel_r], ['human speed', 'robot speed'])


# TTC + REL_SPD#

ax5.set_xlabel('time (s)')
ax5.set_ylabel('TTC (s)')
ax5.plot(ttc_time, ttc_data, 'k+')
ax5.set_ylim(bottom=0)
# ax5.set_ylim(bottom=0, top=17)
ax5.set_xlim(left=min_x, right=max_x)

ax6 = ax5.twinx()
ax6.set_ylabel('relative speed (m/s)')
rel_spd, = ax6.plot(rel_spd_time, rel_spd_data, '-y')
ax6.legend([rel_spd], ['relative speed'])
ax6.set_ylim(bottom=0)

# SEEN_RATIO + SURPRISED

ax7.set_xlabel('time (s)')
ax7.set_ylabel('seen ratio')
ax7.plot(seen_ratio_time, seen_ratio_data, '-k')
surprise, = ax7.plot(surprise_time, surprise_data, 'ro')
ax7.legend([surprise], ['surprise'])
ax7.set_ylim(bottom=0, top=1)
ax7.set_xlim(left=min_x, right=max_x)

if len(sys.argv)>=3:
    left_lim = float(sys.argv[2])
    ax1.set_xlim(left=left_lim)
    ax2.set_xlim(left=left_lim)
    ax5.set_xlim(left=left_lim)
    ax7.set_xlim(left=left_lim)

if len(sys.argv)>=4:
    right_lim = float(sys.argv[3])
    ax1.set_xlim(right=right_lim)
    ax2.set_xlim(right=right_lim)
    ax5.set_xlim(right=right_lim)
    ax7.set_xlim(right=right_lim)

#fig.tight_layout()

plt.show()

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

list_vel_h_data = []
list_vel_h_time = []

list_vel_r_data = []
list_vel_r_time = []

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

list_rel_spd_data = []
list_rel_spd_time = []

value_state = -1

for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")

        if mylist[2] == 'SUPERVISOR' or mylist[2] == 'HUMAN_MODEL' or mylist[2] == 'LOG' or mylist[2] == 'CONFLICT_MANAGER':
            if mylist[3] == 'DIST':
                list_dist_data.append(float(mylist[4]))
                list_dist_time.append(float(mylist[5]))

            elif mylist[3] == 'FIRST':
                list_first_data.append(float(mylist[4]))
                list_first_time.append(float(mylist[5]))

            elif mylist[3] == "PATH":
                list_path_data.append(float(mylist[4]))
                list_path_time.append(float(mylist[5]))

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

            elif mylist[3] == 'VEL_H':
                list_vel_h_data.append(float(mylist[4]))
                list_vel_h_time.append(float(mylist[0]))

            elif mylist[3] == 'VEL_R':
                list_vel_r_data.append(float(mylist[4]))
                list_vel_r_time.append(float(mylist[0]))

            elif mylist[3] == 'REL_SPD':
                list_rel_spd_data.append(float(mylist[4]))
                list_rel_spd_time.append(float(mylist[5]))

f.close()

fig, (ax1, ax2, ax5) = plt.subplots(3, 1)
plt.subplots_adjust(left=0.10, right=0.91, bottom=0.10, top=0.98, hspace = 0.24)

# PATHS #
#ax1.set_xlabel('time (s)')
ax1.set_ylabel('path length (m)')
ax1.plot(list_path_time, list_path_data, 'b+')
ax1.plot(list_first_time, list_first_data, 'ro')
ax1.plot(list_state_exec_time, list_state_exec_data, 'gs')
ax1.plot(list_state_approach_time, list_state_approach_data, 'ys')
ax1.plot(list_state_blocked_time, list_state_blocked_data, 'rs')

# DIST + VELS #
color = 'tab:grey'
#ax2.set_xlabel('time (s)')
ax2.set_ylabel('human-robot \n distance (m)')
ax2.plot(list_dist_time, list_dist_data, '--', color=color)
ax2.set_xlim(left=ax1.get_xbound()[0], right=ax1.get_xbound()[1])
ax2.set_ylim(bottom=0)

ax3 = ax2.twinx()
ax3.set_ylabel('speeds (m/s)')
#ax3.set_ylabel('vel_h (m/s)', color=color)
vel_h, = ax3.plot(list_vel_h_time, list_vel_h_data, '-b')
ax3.set_ylim(bottom=0)

ax4 = ax2.twinx()
#ax4.set_ylabel('vel_r (m/s)', color=color)
vel_r, = ax4.plot(list_vel_r_time, list_vel_r_data, '-r')
ax4.set_ylim(bottom=0)

maxi_vel = max(ax3.get_ybound()[1], ax4.get_ybound()[1])
maxi_vel = 1.5
ax3.set_ylim(top=maxi_vel)
ax4.set_ylim(top=maxi_vel)

ax2.legend([vel_h, vel_r], ['human speed', 'robot speed'])


# TTC + REL_SPD#
ax5.set_xlabel('time (s)')
ax5.set_ylabel('TTC (s)')
ax5.plot(list_ttc_time, list_ttc_data, 'k+')
ax5.set_ylim(bottom=0, top=6)
ax5.set_xlim(left=ax1.get_xbound()[0], right=ax1.get_xbound()[1])

ax6 = ax5.twinx()
ax6.set_ylabel('relative speed (m/s)')
rel_spd, = ax6.plot(list_rel_spd_time, list_rel_spd_data, '-y')
ax6.set_ylim(bottom=0)


if len(sys.argv)>=3:
    left_lim = float(sys.argv[2])
    ax1.set_xlim(left=left_lim)
    ax2.set_xlim(left=left_lim)
    ax5.set_xlim(left=left_lim)

if len(sys.argv)>=4:
    right_lim = float(sys.argv[3])
    ax1.set_xlim(right=right_lim)
    ax2.set_xlim(right=right_lim)
    ax5.set_xlim(right=right_lim)

#fig.tight_layout()

plt.show()

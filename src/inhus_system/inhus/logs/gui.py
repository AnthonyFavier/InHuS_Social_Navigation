from bokeh.layouts import layout
from bokeh.models import Div, Slider, LinearAxis, Range1d, CheckboxGroup, CheckboxButtonGroup, Button, HoverTool
from bokeh.models.callbacks import CustomJS
from bokeh.plotting import figure, show, output_file
import sys

######################################################################################################

filename = "inhus_logs/log.txt"
f = open(filename, "r")

# start
string = f.readline()

path_data = []
path_time = []

first_data = []
first_time = []

state_exec_data = []
state_exec_time = []

state_approach_data = []
state_approach_time = []

state_blocked_data = []
state_blocked_time = []

dist_data = []
dist_time = []

vel_h_data = []
vel_h_time = []

vel_r_data = []
vel_r_time = []

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

def computeYMax(list):
    return max(list)*1.05

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

max_vel = max(computeYMax(vel_h_data), computeYMax(vel_r_data))
max_rel_vel = computeYMax(rel_spd_data)
max_tcc = computeYMax(ttc_data)

######################################################################################################

height = 180
width = 800
common_tools = "pan,wheel_zoom,box_zoom,save,crosshair"
TOOLTIPS = [
    ("(x,y)", "($x, $y)"),
]

# output_file("js_on_change.html")

# Figures
p1 = figure(x_range=(min_x, max_x), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p1.toolbar.logo = None
p1.toolbar.autohide = True
p1.add_tools(HoverTool(tooltips=TOOLTIPS))
p1.yaxis.axis_label = "Path length (m)"
path = p1.cross(x=path_time, y=path_data, size=8, line_width=2, legend_label="path length")
first = p1.circle(x=first_data, y=first_time, size=8, color='red', legend_label="first path length")
state_exec = p1.square(x=state_exec_time, y=state_exec_data, size=8, color='green', legend_label="EXEC state")
state_approach = p1.square(x=state_approach_time, y=state_approach_data, size=8, color='orange', legend_label="APPROACH state")
state_blocked = p1.square(x=state_blocked_time, y=state_blocked_data, size=8, color='red', legend_label="BLOCKED state")
p1.legend.visible=False
p1.legend.margin = -100

p2 = figure(x_range=(min_x, max_x), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p2.toolbar.logo = None
p2.toolbar.autohide = True
p2.add_tools(HoverTool(tooltips=TOOLTIPS))
p2.yaxis.axis_label = "Human-Robot Distance (m)"
p2.extra_y_ranges = {"foo": Range1d(start=0, end=max_vel)}
p2.add_layout(LinearAxis(y_range_name="foo", axis_label="Speeds (m/s)"), 'right')
dist = p2.line(x=dist_time, y=dist_data, line_width=3, color='gray', line_dash="dashed", legend_label="distance")
vel_h = p2.line(x=vel_h_time, y=vel_h_data, line_width=3, color='blue', y_range_name="foo", legend_label="speed h")
vel_r = p2.line(x=vel_r_time, y=vel_r_data, line_width=3, color='red', y_range_name="foo", legend_label="speed r")
p2.legend.visible=False
p2.legend.margin = -100

p3 = figure(x_range=(min_x, max_x), y_range=(0, max_tcc), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p3.toolbar.logo = None
p3.toolbar.autohide = True
p3.add_tools(HoverTool(tooltips=TOOLTIPS))
p3.yaxis.axis_label = "TTC (s)"
p3.extra_y_ranges = {"foo2" : Range1d(start=0, end=max_rel_vel)}
p3.add_layout(LinearAxis(y_range_name="foo2", axis_label="Relative speed (m/s)"), 'right')
ttc = p3.cross(x=ttc_time, y=ttc_data, size=8, color='black', legend_label="TTC")
rel_spd = p3.line(x=rel_spd_time, y=rel_spd_data, line_width=3, color='orange', y_range_name="foo2", legend_label="rel speed")
p3.legend.visible=False
p3.legend.margin = -100

p4 = figure(x_range=(min_x, max_x), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p4.toolbar.logo = None
p4.toolbar.autohide = True
p4.add_tools(HoverTool(tooltips=TOOLTIPS, mode='vline'))
p4.xaxis.axis_label = "Time (s)"
p4.yaxis.axis_label = "Seen ratio  / Surprised"
seen_ratio = p4.line(x=seen_ratio_time, y=seen_ratio_data, line_width=3, color='black', legend_label="seen ratio")
surprise = p4.circle(x=surprise_time, y=surprise_data, size=8, color='red', legend_label="surprised")
p4.legend.visible=False
p4.legend.margin = -100

# Widgets
div = Div(text="Salut")

slider_height = Slider(title="height", start = 100, end = 500, step = 1, value = height)
slider_height.js_link("value", p1, "height")
slider_height.js_link("value", p2, "height")
slider_height.js_link("value", p3, "height")
slider_height.js_link("value", p4, "height")

slider_width = Slider(title="width", start = 100, end = 1200, step = 1, value = width)
slider_width.js_on_change("value",
    CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4, div=div),
        code="""
            p1.frame_width = cb_obj.value
            p1.height = p1.height+1
            p1.height = p1.height-1
            p2.frame_width = cb_obj.value
            p3.frame_width = cb_obj.value
            p4.frame_width = cb_obj.value
            div.text = cb_obj.value.toString()
        """)
)

legend_button = CheckboxButtonGroup(labels=["Show legends"], active=[], width_policy="min")
legend_button.js_on_change(
    "active", 
    CustomJS(args=dict(l1=p1.legend[0], l2=p2.legend[0], l3=p3.legend[0], l4=p4.legend[0]),
    code="""
        const active = cb_obj.active
        if(active.length){
            l1.visible = true
            l1.margin = 10
            l2.visible = true
            l2.margin = 10
            l3.visible = true
            l3.margin = 10
            l4.visible = true
            l4.margin = 10
        }else{
            l1.visible = false
            l1.margin = -100
            l2.visible = false
            l2.margin = -100
            l3.visible = false
            l3.margin = -100
            l4.visible = false
            l4.margin = -100
        }
    """)
)

reset_button = Button(label="Reset plots", button_type="success", width_policy="min")
reset_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4), 
    code="""
    p1.reset.emit()
    p2.reset.emit()
    p3.reset.emit()
    p4.reset.emit()
    """)
)

reset_size_button = Button(label="Reset plot sizes", button_type="success", width_policy="min")
reset_size_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4, h=height, w=width, sh=slider_height, sw=slider_width), 
    code="""
    p1.frame_width = w
    p1.height = h
    p2.frame_width = w
    p2.height = h
    p3.frame_width = w
    p3.height = h
    p4.frame_width = w
    p4.height = h
    sh.value = h
    sw.value = w
    """)
)

# create layout
layout = layout(
    [
        [slider_height, legend_button],
        [slider_width, reset_button, reset_size_button],
        [p1],
        [p2],
        [p3],
        [p4],
    ]
)

# show result
show(layout)
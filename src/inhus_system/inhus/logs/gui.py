from bokeh.io import curdoc
from bokeh.layouts import layout, column, row
from bokeh.models import Div, Slider, LinearAxis, Range1d, CheckboxGroup, CheckboxButtonGroup, Button, HoverTool, RadioButtonGroup, ColumnDataSource, TextInput
from bokeh.models.callbacks import CustomJS
from bokeh.plotting import figure, show, output_file
import sys

######################################################################################################

path_data = []
path_time = []
path_source = ColumnDataSource()

first_data = []
first_time = []
first_source = ColumnDataSource()

state_exec_data = []
state_exec_time = []
state_exec_source = ColumnDataSource()

state_approach_data = []
state_approach_time = []
state_approach_source = ColumnDataSource()

state_blocked_data = []
state_blocked_time = []
state_blocked_source = ColumnDataSource()

dist_data = []
dist_time = []
dist_source = ColumnDataSource()

vel_h_data = []
vel_h_time = []
vel_h_source = ColumnDataSource()

vel_r_data = []
vel_r_time = []
vel_r_source = ColumnDataSource()

ttc_data = []
ttc_time = []
ttc_source = ColumnDataSource()

rel_spd_data = []
rel_spd_time = []
rel_spd_source = ColumnDataSource()

surprise_data = []
surprise_time = []
surprise_source = ColumnDataSource()

seen_ratio_data = []
seen_ratio_time = []
seen_ratio_source = ColumnDataSource()

min_x_default = 0
max_x_default = 0
max_vel = 0
max_rel_vel = 0
max_tcc = 0

######################################################################################################

def readDataFromFile(filename):
    print("read Data")
    f = open(filename, "r")

    value_state = -1

    path_data.clear()
    path_time.clear()
    first_data.clear()
    first_time.clear()
    state_exec_data.clear()
    state_exec_time.clear()
    state_approach_data.clear()
    state_approach_time.clear()
    state_blocked_data.clear()
    state_blocked_time.clear()
    dist_data.clear()
    dist_time.clear()
    vel_h_data.clear()
    vel_h_time.clear()
    vel_r_data.clear()
    vel_r_time.clear()
    ttc_data.clear()
    ttc_time.clear()
    rel_spd_data.clear()
    rel_spd_time.clear()
    surprise_data.clear()
    surprise_time.clear()
    seen_ratio_data.clear()
    seen_ratio_time.clear()

    # start
    string = f.readline()

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

def treatData():
    print("treat data")
    global min_x_default
    global max_x_default
    global max_vel
    global max_rel_vel
    global max_tcc

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
    min_x_default = min(dist_time[0], vel_h_time[0], vel_r_time[0], rel_spd_time[0], seen_ratio_time[0])
    max_x_default = max(dist_time[-1], vel_h_time[-1], vel_r_time[-1], rel_spd_time[-1], seen_ratio_time[-1])
    max_vel = max(computeYMax(vel_h_data), computeYMax(vel_r_data))
    max_rel_vel = computeYMax(rel_spd_data)
    max_tcc = computeYMax(ttc_data)

def updateColumnDataSource():
    print("update source")
    global path_source
    global first_source
    global state_exec_source
    global state_approach_source
    global state_blocked_source
    global dist_source
    global vel_h_source
    global vel_r_source
    global ttc_source
    global rel_spd_source
    global surprise_source
    global seen_ratio_source

    path_source.data = dict(x=path_time, y=path_data)
    first_source.data = dict(x=first_time, y=first_data)
    state_exec_source.data = dict(x=state_exec_time, y=state_exec_data)
    state_approach_source.data = dict(x=state_approach_time, y=state_approach_data)
    state_blocked_source.data = dict(x=state_blocked_time, y=state_blocked_data)
    dist_source.data = dict(x=dist_time, y=dist_data)
    vel_h_source.data = dict(x=vel_h_time, y=vel_h_data)
    vel_r_source.data = dict(x=vel_r_time, y=vel_r_data)
    ttc_source.data = dict(x=ttc_time, y=ttc_data)
    rel_spd_source.data = dict(x=rel_spd_time, y=rel_spd_data)
    surprise_source.data = dict(x=surprise_time, y=surprise_data)
    seen_ratio_source.data = dict(x=seen_ratio_time, y=seen_ratio_data)

######################################################################################################

readDataFromFile("inhus_logs/log.txt")
treatData()
updateColumnDataSource()

height = 180
width = 800
muted_alpha=0.2
margin=-100
default_click_policy = "mute"
common_tools = "pan,wheel_zoom,box_zoom,save,crosshair"
TOOLTIPS = [("(x,y)", "($x, $y)")]

######################################################################################################
#############
## FIGURES ##
#############

p1 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p1.toolbar.logo = None
p1.toolbar.autohide = True
p1.add_tools(HoverTool(tooltips=TOOLTIPS))
p1.yaxis.axis_label = "Path length (m)"
path = p1.cross('x', 'y', source=path_source, size=8, line_width=2, muted_alpha=muted_alpha, legend_label="path length")
first = p1.circle('x', 'y', source=first_source, size=8, color='red', muted_alpha=muted_alpha, legend_label="first path length")
state_exec = p1.square('x', 'y', source=state_exec_source, size=8, color='green', muted_alpha=muted_alpha, legend_label="EXEC state")
state_approach = p1.square('x', 'y', source=state_approach_source, size=8, color='orange', muted_alpha=muted_alpha, legend_label="APPROACH state")
state_blocked = p1.square('x', 'y', source=state_blocked_source, size=8, color='red', muted_alpha=muted_alpha, legend_label="BLOCKED state")
p1.legend.visible=False
p1.legend.margin = margin
p1.legend.click_policy=default_click_policy

p2 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p2.toolbar.logo = None
p2.toolbar.autohide = True
p2.add_tools(HoverTool(tooltips=TOOLTIPS))
p2.yaxis.axis_label = "H-R Distance (m)"
p2.extra_y_ranges = {"foo": Range1d(start=0, end=max_vel)}
p2.add_layout(LinearAxis(y_range_name="foo", axis_label="Speeds (m/s)"), 'right')
dist = p2.line('x', 'y', source=dist_source, line_width=3, color='gray', muted_alpha=muted_alpha, line_dash="dashed", legend_label="distance", )
vel_h = p2.line('x', 'y', source=vel_h_source, line_width=3, color='blue', muted_alpha=muted_alpha, y_range_name="foo", legend_label="speed h")
vel_r = p2.line('x', 'y', source=vel_r_source, line_width=3, color='red', muted_alpha=muted_alpha, y_range_name="foo", legend_label="speed r")
p2.legend.visible=False
p2.legend.margin = margin
p2.legend.click_policy=default_click_policy

p3 = figure(x_range=(min_x_default, max_x_default), y_range=(0, max_tcc), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p3.toolbar.logo = None
p3.toolbar.autohide = True
p3.add_tools(HoverTool(tooltips=TOOLTIPS))
p3.yaxis.axis_label = "TTC (s)"
p3.extra_y_ranges = {"foo2" : Range1d(start=0, end=max_rel_vel)}
p3.add_layout(LinearAxis(y_range_name="foo2", axis_label="Relative speed (m/s)"), 'right')
ttc = p3.cross('x', 'y', source=ttc_source, size=8, color='black', muted_alpha=muted_alpha, legend_label="TTC")
rel_spd = p3.line('x', 'y', source=rel_spd_source, line_width=3, color='orange', muted_alpha=muted_alpha, y_range_name="foo2", legend_label="rel speed")
p3.legend.visible=False
p3.legend.margin = margin
p3.legend.click_policy=default_click_policy

p4 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height)
p4.toolbar.logo = None
p4.toolbar.autohide = True
p4.add_tools(HoverTool(tooltips=TOOLTIPS, mode='vline'))
p4.xaxis.axis_label = "Time (s)"
p4.yaxis.axis_label = "Seen ratio  / Surprised"
seen_ratio = p4.line('x', 'y', source=seen_ratio_source, line_width=3, color='black', muted_alpha=muted_alpha, legend_label="seen ratio")
surprise = p4.circle('x', 'y', source=surprise_source, size=8, color='red', muted_alpha=muted_alpha, legend_label="surprised")
p4.legend.visible=False
p4.legend.margin = margin
p4.legend.click_policy=default_click_policy

######################################################################################################
#############
## WIDGETS ##
#############

# Div General tetxts
plot_size_div = Div(text="<b>Plot size:</b>")
legend_div = Div(text="<b>Legend:</b>")
other_div = Div(text="<b>Other:</b>")

# Slider Height
slider_height = Slider(title="height", start = 100, end = 500, step = 1, value = height)
slider_height.js_link("value", p1, "height")
slider_height.js_link("value", p2, "height")
slider_height.js_link("value", p3, "height")
slider_height.js_link("value", p4, "height")

# Slider Width
slider_width = Slider(title="width", start = 100, end = 1800, step = 1, value = width)
slider_width.js_on_change("value",
    CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4),
        code="""
            p1.frame_width = cb_obj.value
            p2.frame_width = cb_obj.value
            p3.frame_width = cb_obj.value
            p4.frame_width = cb_obj.value
            p4.height = p1.height+1
            p4.height = p1.height-1
        """))

# Button Show Legend
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
    """))

# Button Reset Size
reset_size_button = Button(label="Reset plot sizes", button_type="primary", width_policy="min")
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
    """))

# RadioButton Hide/Mute Legend
hide_mute_button_div = Div(text="Legend click policy:")
hide_mute_button = RadioButtonGroup(labels=["Mute", "Hide"], active=0, width_policy="min")
hide_mute_button.js_on_click(CustomJS(args=dict(l1=p1.legend[0], l2=p2.legend[0], l3=p3.legend[0], l4=p4.legend[0]), 
    code="""
    if(cb_obj.active==0)
    {
        l1.click_policy="mute"
        l2.click_policy="mute"
        l3.click_policy="mute"
        l4.click_policy="mute"
    }
    else
    {
        l1.click_policy="hide"
        l2.click_policy="hide"
        l3.click_policy="hide"
        l4.click_policy="hide"
    }
    """))

# Button Update Data 
def updateFigureRange(min_x=None, max_x=None):
    print("update figure range")
    global p1, p2, p3, p4

    if min_x==None and max_x==None:
        min_x = min_x_default
        max_x = max_x_default

        p1.x_range.start=min_x
        p1.x_range.end=max_x
        p2.x_range.start=min_x
        p2.x_range.end=max_x
        p3.x_range.start=min_x
        p3.x_range.end=max_x
        p4.x_range.start=min_x
        p4.x_range.end=max_x
    elif max_x==None:
        p1.x_range.start=min_x
        p2.x_range.start=min_x
        p3.x_range.start=min_x
        p4.x_range.start=min_x
    elif min_x==None:
        p1.x_range.end=max_x
        p2.x_range.end=max_x
        p3.x_range.end=max_x
        p4.x_range.end=max_x
    else:
        p1.x_range.start=min_x
        p1.x_range.end=max_x
        p2.x_range.start=min_x
        p2.x_range.end=max_x
        p3.x_range.start=min_x
        p3.x_range.end=max_x
        p4.x_range.start=min_x
        p4.x_range.end=max_x
def updateData(event=None):
    readDataFromFile("inhus_logs/log.txt")
    treatData()
    updateColumnDataSource()
    updateFigureRange()
update_data_button = Button(label="Update data", button_type="success", width_policy="min")
update_data_button.on_click(updateData)

# TextInput t_min
t_min_input = TextInput(value="{:.1f}".format(min_x_default), title="Time min:", width=70)
def update_t_min(attr,old,new):
    t_min = 0.0
    try:
        t_min = float(new)
        updateFigureRange(min_x=t_min)
    except:
        print("Wrong input ...")
t_min_input.on_change("value", update_t_min)

# TextInput t_max
t_max_input = TextInput(value="{:.1f}".format(max_x_default), title="Time max:", width=70)
def update_t_max(attr,old,new):
    t_max = 0.0
    try:
        t_max = float(new)
        updateFigureRange(max_x=t_max)
    except:
        print("Wrong input ...")
t_max_input.on_change("value", update_t_max)

# Button Reset Plots
reset_button = Button(label="Reset plots", button_type="primary", width_policy="min")
def reset_buttonCB(event):
    global t_min_input
    global t_max_input
  
    t_min_input.value="{:.1f}".format(min_x_default)
    t_max_input.value="{:.1f}".format(max_x_default)
reset_button.on_click(reset_buttonCB)
reset_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4), 
    code="""
    p1.reset.emit()
    p2.reset.emit()
    p3.reset.emit()
    p4.reset.emit()
    """))

# Button Reset Plots Range
reset_range_button = Button(label="Reset range plots", button_type="primary", width_policy="min")
def reset_range_buttonCB(event):
    global t_min_input
    global t_max_input
  
    t_min=None
    t_max=None
    try:
        t_min=float(t_min_input.value)
    except:
        print("reset range min wrong input ...")
    try:
        t_max=float(t_max_input.value)
    except:
        print("reset range max wrong input ...")

    updateFigureRange(t_min, t_max)
reset_range_button.on_click(reset_range_buttonCB)
reset_range_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4), 
    code="""
    p1.reset.emit()
    p2.reset.emit()
    p3.reset.emit()
    p4.reset.emit()
    """))

######################################################################################################
############
## LAYOUT ##
############

plot_size_column = column(plot_size_div, slider_height, slider_width, reset_size_button)
legend_column = column(legend_div, legend_button, hide_mute_button_div, hide_mute_button)
other_column = column(other_div, reset_button, update_data_button)

t_range_row = row(t_min_input, t_max_input)
range_column = column(t_range_row, reset_range_button)

layout = layout(
    [
        [plot_size_column, legend_column, other_column, range_column],
        [p1],
        [p2],
        [p3],
        [p4],
    ])

######################################################################################################
##########
## SHOW ##
##########

# show result
# show(layout)

curdoc().add_root(layout)
curdoc().title = "InHuS Logs"
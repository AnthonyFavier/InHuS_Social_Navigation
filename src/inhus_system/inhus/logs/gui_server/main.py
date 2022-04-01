from os import times
from bokeh.io import curdoc
from bokeh.layouts import layout, column, row
from bokeh.models import HoverTool, WheelZoomTool
from bokeh.models import Div, Slider, CheckboxGroup, CheckboxButtonGroup, Button, RadioButtonGroup, TextInput, PreText
from bokeh.models import LinearAxis, Range1d, ColumnDataSource, ColorBar, CDSView, IndexFilter, CustomJSFilter
from bokeh.models.callbacks import CustomJS
from bokeh.plotting import figure, show, output_file
from bokeh.palettes import Spectral6, Turbo256
from bokeh.transform import LinearColorMapper
from bokeh.themes import built_in_themes
import sys
import numpy as np
import math

absolute_path = ""
map_name = "passage_hri"
if len(sys.argv) >= 3:
    absolute_path = sys.argv[1] + "/"
    map_name = sys.argv[2]

######################################################################################################
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

######################################################################################################
######################################################################################################
######################################################################################################
############
## HELPER ##
############

def findIndexWithVal(list, val, f_i):
    margin = 0.05
    if len(list)==0:
        return None
    else:
        m = math.floor(len(list)/2)
        if list[m] >= val-margin and list[m] <= val+margin:
            return f_i+m
        elif val < list[m]-margin:
            return findIndexWithVal(list[:m-1], val, f_i)
        elif val > list[m]+margin:
            return findIndexWithVal(list[m+1:], val, f_i+m+1)

def update_t_g(t_min=None, t_max=None):
    global t_min_g
    global t_max_g

    if t_min!=None:
        t_min_g = t_min
    if t_max!=None:
        t_max_g = t_max
    updateInputValue(t_min_g, t_max_g)
    updateFigureRange(min_x=t_min_g, max_x=t_max_g)
    updateFilters()
    updateMapper()

def updateInputValue(t_min=None, t_max=None):
    global t_min_input
    global t_max_input

    t_min_new = t_min_input.value
    t_max_new = t_max_input.value
    if t_min!=None:
        t_min_new = t_min
    if t_max!=None:
        t_max_new = t_max
    
    t_min_input.value = "{:.1f}".format(t_min_new)
    t_max_input.value = "{:.1f}".format(t_max_new)

def updateFigureRange(min_x=None, max_x=None):
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

def updateMapper():
    global mapper
    mapper.low = t_min_g
    mapper.high = t_max_g

def updateFilters():
    global view_h
    global view_r

    date_filter_h = CustomJSFilter(args=dict(t_min_g=t_min_g, t_max_g=t_max_g, source_data=path_H_source,), code="""
    let start=t_min_g
    let end=t_max_g;
    let dates = source_data.data['stamp'];
    let indices = [];
    for (var i = 0; i <= dates.length; i++){
        if (dates[i] >= start && dates[i] <= end) indices.push(i);
    }
    return indices;
    """)

    date_filter_r = CustomJSFilter(args=dict(t_min_g=t_min_g, t_max_g=t_max_g, source_data=path_R_source,), code="""
    let start=t_min_g
    let end=t_max_g;
    let dates = source_data.data['stamp'];
    let indices = [];
    for (var i = 0; i <= dates.length; i++){
        if (dates[i] >= start && dates[i] <= end) indices.push(i);
    }
    return indices;
    """)

    view_h.filters = [resol_filter_h, date_filter_h]
    view_r.filters = [resol_filter_r, date_filter_r]

######################################################################################################
######################################################################################################

def readGraphData():
    readDataFromFile("inhus_logs/log.txt")
    treatData()
    updateColumnDataSource()

def readDataFromFile(filename):
    filename = absolute_path + filename
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
    global min_x_default
    global max_x_default
    global max_vel
    global max_rel_vel
    global t_min_g
    global t_max_g

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
    min_x_default = min(dist_time[0], vel_h_time[0], vel_r_time[0], rel_spd_time[0])
    # min_x_default = min(dist_time[0], vel_h_time[0], vel_r_time[0], rel_spd_time[0], seen_ratio_time[0])
    max_x_default = max(dist_time[-1], vel_h_time[-1], vel_r_time[-1], rel_spd_time[-1])
    # max_x_default = max(dist_time[-1], vel_h_time[-1], vel_r_time[-1], rel_spd_time[-1], seen_ratio_time[-1])
    t_min_g = min_x_default
    t_max_g = max_x_default
    max_vel = max(computeYMax(vel_h_data), computeYMax(vel_r_data))
    max_rel_vel = computeYMax(rel_spd_data)

def updateColumnDataSource():
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
######################################################################################################

readGraphData()

height = 170
width = 800
muted_alpha=0.2
margin=-100
default_click_policy = "mute"
common_tools = "pan,wheel_zoom,save,crosshair"
TOOLTIPS = [("(x,y)", "($x, $y)")]

######################################################################################################
############################################ GRAPHS ##################################################
######################################################################################################
#############
## FIGURES ##
#############

p1 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height, output_backend="webgl")
p1.toolbar.logo = None
p1.toolbar.autohide = False
p1.toolbar_location = 'left'
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

p2 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height, output_backend="webgl")
p2.toolbar.logo = None
p2.toolbar.autohide = False
p2.toolbar_location = 'left'
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

p3 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height, output_backend="webgl")
p3.toolbar.logo = None
p3.toolbar.autohide = False
p3.toolbar_location = 'left'
p3.add_tools(HoverTool(tooltips=TOOLTIPS))
p3.yaxis.axis_label = "TTC (s)"
p3.y_range = Range1d(0,10);
p3.extra_y_ranges = {"foo2" : Range1d(start=0, end=max_rel_vel)}
p3.add_layout(LinearAxis(y_range_name="foo2", axis_label="Relative speed (m/s)"), 'right')
ttc = p3.cross('x', 'y', source=ttc_source, size=8, color='black', muted_alpha=muted_alpha, legend_label="TTC")
rel_spd = p3.line('x', 'y', source=rel_spd_source, line_width=3, color='orange', muted_alpha=muted_alpha, y_range_name="foo2", legend_label="rel speed")
p3.legend.visible=False
p3.legend.margin = margin
p3.legend.click_policy=default_click_policy

p4 = figure(x_range=(min_x_default, max_x_default), tools=common_tools, active_scroll="wheel_zoom", frame_width=width, height=height, output_backend="webgl")
p4.toolbar.logo = None
p4.toolbar.autohide = False
p4.toolbar_location = 'left'
p4.add_tools(HoverTool(tooltips=TOOLTIPS, mode='vline'))
p4.xaxis.axis_label = "Time (s)"
p4.yaxis.axis_label = "Seen ratio  / Surprised"
seen_ratio = p4.line('x', 'y', source=seen_ratio_source, line_width=3, color='black', muted_alpha=muted_alpha, legend_label="seen ratio")
surprise = p4.circle('x', 'y', source=surprise_source, size=8, color='red', muted_alpha=muted_alpha, legend_label="surprised")
p4.legend.visible=False
p4.legend.margin = margin
p4.legend.click_policy=default_click_policy

######################################################################################################
######################################## COLORED PATH ################################################
######################################################################################################
#############
## FIGURES ##
#############

# Global var 
img_file = None
img_size = None
img_resolution = None
img_offset = None
img_angle = None

path_H_stamp = []
path_H_x = []
path_H_y = []
path_H_theta = []
path_H_source = ColumnDataSource()

path_R_stamp = []
path_R_x = []
path_R_y = []
path_R_theta = []
path_R_source = ColumnDataSource()

def readPathData():
    getMapInfo(map_name)
    readPoseData()
    updatePoseSource()

def getMapInfo(map_name):
    global img_file
    global img_size
    global img_resolution
    global img_offset
    global img_angle

    filename = absolute_path + "gui_server/static/" + map_name + ".txt"
    f = open(filename, "r")
    for line in f:
        if 'str' in line:
            break
        if line != "\n":
            line = line.replace("\n","")
            mylist = line.split(" ")

            if mylist[0] == 'image:':
                img_file = mylist[1]
            elif mylist[0] == 'size:':
                img_size = [int(mylist[1]), int(mylist[2])]
            elif mylist[0] == 'resolution:':
                img_resolution = float(mylist[1])
            elif mylist[0] == 'offset:':
                img_offset = [float(mylist[1]), float(mylist[2])]
            elif mylist[0] == 'angle:':
                img_angle = float(mylist[1])
    f.close()

    # Scale image
    last_img_size = img_size
    last_img_resolution = img_resolution
    if img_size[0] > max_img_x and img_size[1] > max_img_y :
        if img_size[0]>img_size[1]:
            r = img_size[0]/max_img_x
            last_img_resolution *= r
            last_img_size[1] = int(last_img_size[1]/r)
            last_img_size[0] = int(max_img_x)
        else:
            r = img_size[1]/max_img_y
            last_img_resolution *= r
            last_img_size[0] = int(last_img_size[0]/r)
            last_img_size[1] = int(max_img_y)
    elif img_size[1] > max_img_y:
        r = img_size[1]/max_img_y
        last_img_resolution *= r
        last_img_size[0] = int(last_img_size[0]/r)
        last_img_size[1] = int(max_img_y)
    elif img_size[0] > max_img_x:
        r = img_size[0]/max_img_x
        last_img_resolution *= r
        last_img_size[1] = int(last_img_size[1]/r)
        last_img_size[0] = int(max_img_x)
    img_resolution = last_img_resolution
    img_size = last_img_size

    # angle
    # x = cos(a)x' - sin(a)y'
    # y = sin(a)x' + cos(a)y'
    tmp = [last_img_size[0]*img_resolution/2 + img_offset[0], last_img_size[1]*img_resolution/2 + img_offset[1]]
    img_offset = [math.cos(img_angle)*tmp[0] - math.sin(img_angle)*tmp[1], math.sin(img_angle)*tmp[0] + math.cos(img_angle)*tmp[1]]

def readPoseData():
    filename = absolute_path + "inhus_logs/poseLog.txt"
    f = open(filename, "r")

    path_H_stamp.clear()
    path_H_x.clear()
    path_H_y.clear()
    path_H_theta.clear()

    path_R_stamp.clear()
    path_R_x.clear()
    path_R_y.clear()
    path_R_theta.clear()

    #  remove first line
    string = f.readline()

    for line in f:
        if 'str' in line:
            break
        if line != "\n":
            line = line.replace("\n","")
            mylist = line.split(" ")

            # x = cos(a)x' - sin(a)y'
            # y = sin(a)x' + cos(a)y'

            px=float(mylist[3])
            py=float(mylist[4])
            if mylist[2] == 'H':
                path_H_stamp.append(float(mylist[0]))
                path_H_x.append(math.cos(img_angle)*px - math.sin(img_angle)*py)
                path_H_y.append(math.sin(img_angle)*px + math.cos(img_angle)*py)
                path_H_theta.append(float(mylist[5])+img_angle)

            elif mylist[2] == 'R':
                path_R_stamp.append(float(mylist[0]))
                path_R_x.append(math.cos(img_angle)*px - math.sin(img_angle)*py)
                path_R_y.append(math.sin(img_angle)*px + math.cos(img_angle)*py)
                path_R_theta.append(float(mylist[5])+img_angle)
    f.close()

def updatePoseSource():
    path_H_source.data = dict(x=path_H_x, y=path_H_y, theta=path_H_theta, stamp=path_H_stamp)
    path_R_source.data = dict(x=path_R_x, y=path_R_y, theta=path_R_theta, stamp=path_R_stamp)

#########################

max_img_x=840
max_img_y=690

readPathData()

resol = 1
resol_filter_h = IndexFilter(np.arange(0, len(path_H_source.data["x"]), resol))
resol_filter_r = IndexFilter(np.arange(0, len(path_R_source.data["x"]), resol))

view_h = CDSView(source=path_H_source, filters=[])
view_r = CDSView(source=path_R_source, filters=[])
updateFilters()

mapper = LinearColorMapper(palette=Turbo256, low=t_min_g, high=t_max_g)
colors = {'field': 'stamp', 'transform': mapper}

TOOLTIPS_BIS = [("index", "$index"), ("time", "@stamp")]
hover_h_source = ColumnDataSource(dict(x=[], y=[], theta=[], stamp=[]))
hover_r_source = ColumnDataSource(dict(x=[], y=[], theta=[], stamp=[]))
previous_source = ColumnDataSource({'x':[0], 'y':[0]})
hover_pathCB = CustomJS(args=dict(hover_h_source=hover_h_source, hover_r_source=hover_r_source, path_H_source=path_H_source, path_R_source=path_R_source, previous_len=previous_source), code="""
    let indices = cb_data['index'].indices
    if(indices.length > 0)
    {
        let index = indices.at(-1)

        let data_h = {
        'x':[path_H_source.data['x'][index]],
        'y':[path_H_source.data['y'][index]],
        'theta':[path_H_source.data['theta'][index]-1.571],
        'stamp':[path_H_source.data['stamp'][index]]
        }
        hover_h_source.data = data_h
        hover_h_source.change.emit()

        let data_r = {
        'x':[path_R_source.data['x'][index]],
        'y':[path_R_source.data['y'][index]],
        'theta':[path_R_source.data['theta'][index]-1.571],
        'stamp':[path_R_source.data['stamp'][index]]
        }
        //console.log(data_h['theta'][0])
        hover_r_source.data = data_r
        hover_r_source.change.emit()
    }
    else
    {
        if(previous_len.data['x'][0] == 0)
        {
            let data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
            hover_h_source.data = data
            hover_r_source.data = data
            hover_h_source.change.emit()
            hover_r_source.change.emit()
        }
    }
    previous_len.data['x'][0] = indices.length
    previous_len.change.emit()
    """)
hover_path_tool = HoverTool(tooltips=None, names=["path_h", "path_r"], callback = hover_pathCB)

radius_agents = 0.1
p_path = figure(tools="pan,wheel_zoom,save,reset", active_scroll="wheel_zoom", match_aspect=True, frame_width=max_img_x, frame_height=max_img_y-60, output_backend="webgl")
p_path.toolbar.logo = None
p_path.toolbar_location = 'below'
p_path.add_tools(hover_path_tool)
map_img = p_path.image_url(url=['gui_server/static/' + img_file], anchor='center', angle=img_angle, x=img_offset[0], y=img_offset[1], w=img_size[0]*img_resolution, h=img_size[1]*img_resolution)
path_H = p_path.circle('x', 'y', source=path_H_source, name="path_h", color=colors, radius=radius_agents, muted_alpha=muted_alpha, legend_label="path H", view=view_h)
path_R = p_path.circle('x', 'y', source=path_R_source, name="path_r", color=colors, radius=radius_agents, muted_alpha=muted_alpha, legend_label="path R", view=view_r)
highlight_h = p_path.triangle('x', 'y', source=hover_h_source, color=colors, angle="theta", angle_units='rad', size=30, line_color="black", line_width=3)
highlight_r = p_path.triangle('x', 'y', source=hover_r_source, color=colors, angle="theta", angle_units='rad', size=30, line_color="black")
p_path.legend.click_policy = "hide"

color_bar = ColorBar(title="Time (s)", color_mapper=mapper, width=8)
p_path.add_layout(color_bar, 'right')


######################################################################################################
########################################## WIDGETS ###################################################
######################################################################################################

# Div General tetxts
inhus_div = Div(width_policy="auto",
    text="""
    <html>
    <head>
    <style>
    h1 {text-align: center;}
    p {text-align: center;}
    div {text-align: center;}
    </style>
    </head>
    <body>

    <h2>InHuS Log Data Visualization</h2>

    </body>
    """,
    margin=(-10,2,-10,200)
    )
plot_size_div = Div(text="<b>Plot size:</b>")
legend_div = Div(text="<b>Legend:</b>")
other_div = Div(text="<b>Other:</b>")
range_div = Div(text="<b>Range:</b>")
time_colored_path_div = Div(text="<b>Time Colored Paths:</b>")

# Slider Height
height_slider = Slider(title="height", start = 100, end = 500, step = 1, value = height, width=200)
height_slider.js_link("value", p1, "height")
height_slider.js_link("value", p2, "height")
height_slider.js_link("value", p3, "height")
height_slider.js_link("value", p4, "height")

# Slider Width
width_slider = Slider(title="width", start = 100, end = 1800, step = 1, value = width, width=200)
width_slider.js_on_change("value",
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
show_legend_button = CheckboxButtonGroup(labels=["Show legends"], active=[], width_policy="min", align="center")
show_legend_button.js_on_change(
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
reset_plot_size_button = Button(label="Reset plot sizes", button_type="primary", width_policy="min")
reset_plot_size_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4, h=height, w=width, sh=height_slider, sw=width_slider), 
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
hide_mute_button = RadioButtonGroup(labels=["Mute", "Hide"], active=0, width_policy="min", align="center")
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
# update_data_button = Button(label="Update data", button_type="success", width_policy="min", align="center")
# def updateDataCB(event=None):
#     readGraphData()

#     readPathData()

#     # updateFigureRange()
#     updateInputValue(t_min_g, t_max_g)
#     # updateFilters()
#     # updateMapper()
# update_data_button.on_click(updateDataCB)
# update_data_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4, p_path=p_path, h_source=path_H_source, r_source=path_R_source), 
#     code="""
#     p1.reset.emit()
#     p2.reset.emit()
#     p3.reset.emit()
#     p4.reset.emit()
#     p_path.reset.emit()
#     h_source.change.emit()
#     r_source.change.emit()
#     """))

# TextInput t_min
t_min_input = TextInput(value="{:.1f}".format(min_x_default), title="Time min:", width=70)
def t_min_inputCB(attr,old,new):
    try:
        t_min = float(new)
        t_min_input.background = None
        update_t_g(t_min=t_min)
    except:
        print("Wrong input time min ...")
        t_min_input.background = "red"
t_min_input.on_change("value", t_min_inputCB)

# TextInput t_max
t_max_input = TextInput(value="{:.1f}".format(max_x_default), title="Time max:", width=70)
def t_max_inputCB(attr,old,new):
    global t_max_input
    try:
        t_max = float(new)
        t_max_input.background = None
        update_t_g(t_max=t_max)
    except:
        print("Wrong input time max ...")
        t_max_input.background = "red"
t_max_input.on_change("value", t_max_inputCB)

# Button t_min minus
t_min_minus_button = Button(label="-", button_type="default", width_policy="min", margin=(0,2,2,2), align="center")
def t_min_minus_buttonCB():
    update_t_g(t_min=round(t_min_g - 1.0))
t_min_minus_button.on_click(t_min_minus_buttonCB)

# Button t_min plus
t_min_plus_button = Button(label="+", button_type="default", width_policy="min", margin=(2,2,2,2), align="center")
def t_min_plus_buttonCB():
    update_t_g(t_min=round(t_min_g + 1.0))
t_min_plus_button.on_click(t_min_plus_buttonCB)

# Button t_max minus
t_max_minus_button = Button(label="-", button_type="default", width_policy="min", margin=(0,2,2,2), align="center")
def t_max_minus_buttonCB():
    update_t_g(t_max=round(t_max_g - 1.0))
t_max_minus_button.on_click(t_max_minus_buttonCB)

# Button t_max plus
t_max_plus_button = Button(label="+", button_type="default", width_policy="min", margin=(2,2,2,2), align="center")
def t_max_plus_buttonCB():
        update_t_g(t_max=round(t_max_g + 1.0))
t_max_plus_button.on_click(t_max_plus_buttonCB)

# Button set range mvt
set_range_mvt_button = Button(label="Set movement range", button_type="default", width_policy="min", align="center")
def set_range_mvt_buttonCB():
    margin = 1.5

    # Get min t mvt
    t_min_mvt_h = None
    for i, vel in enumerate(vel_h_source.data["y"]):
        if vel != 0.0:
            t_min_mvt_h = vel_h_source.data["x"][i]
            break
    t_min_mvt_r=None
    for i, vel in enumerate(vel_r_source.data["y"]):
        if vel != 0.0:
            t_min_mvt_r = vel_r_source.data["x"][i]
            break
    if t_min_mvt_h != None and t_min_mvt_r != None:
        t_min_mvt = min(t_min_mvt_h, t_min_mvt_r)-margin
    elif t_min_mvt_h != None and t_min_mvt_r == None:
        t_min_mvt = t_min_mvt_h-margin
    elif t_min_mvt_h == None and t_min_mvt_r != None:
        t_min_mvt = t_min_mvt_r-margin
    else:
        return

    # Get max t mvt
    t_max_mvt_h = None
    for i in range(len(vel_h_source.data["y"])-1, -1, -1):
        vel = vel_h_source.data["y"][i]
        if vel != 0.0:
            t_max_mvt_h = vel_h_source.data["x"][i]
            break
    t_max_mvt_r = None
    for i in range(len(vel_r_source.data["y"])-1, -1, -1):
        vel = vel_r_source.data["y"][i]
        if vel != 0.0:
            t_max_mvt_r = vel_r_source.data["x"][i]
            break
    if t_max_mvt_h != None and t_max_mvt_r != None:
        t_max_mvt = max(t_max_mvt_h, t_max_mvt_r)+margin
    elif t_max_mvt_h != None and t_max_mvt_r == None:
        t_max_mvt = t_max_mvt_h+margin
    elif t_max_mvt_h == None and t_max_mvt_r != None:
        t_max_mvt = t_max_mvt_r+margin
    else:
        return

    # Update t range
    update_t_g(t_min_mvt, t_max_mvt)
set_range_mvt_button.on_click(set_range_mvt_buttonCB)

# Button Reset Plots
reset_button = Button(label="Reset plots", button_type="primary", width_policy="min", align="center")
def reset_buttonCB(event):
    updateInputValue(min_x_default, max_x_default)
reset_button.on_click(reset_buttonCB)
reset_button.js_on_click(CustomJS(args=dict(p1=p1, p2=p2, p3=p3, p4=p4), 
    code="""
    p1.reset.emit()
    p2.reset.emit()
    p3.reset.emit()
    p4.reset.emit()
    """))

# Button Reset Plots Range
reset_range_button = Button(label="Reset plots with range", button_type="primary", width_policy="min", align="center")
def reset_range_buttonCB(event):
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

# Button play
play_button = Button(label="Play", button_type="success", width_policy="min", align="center")
periodic_cb = None
playing = False
def play_buttonCBp():
    global i_play
    global hover_path_tool
    global playing
    global time_path_pretext

    # print("step i_play={}".format(i_play))
    hover_h_source.data = {
    'x':[path_H_source.data['x'][i_play]],
    'y':[path_H_source.data['y'][i_play]],
    'theta':[path_H_source.data['theta'][i_play]-1.571],
    'stamp':[path_H_source.data['stamp'][i_play]]
    }

    hover_r_source.data = {
    'x':[path_R_source.data['x'][i_play]],
    'y':[path_R_source.data['y'][i_play]],
    'theta':[path_R_source.data['theta'][i_play]-1.571],
    'stamp':[path_R_source.data['stamp'][i_play]]
    }

    i_play+= 15
    if i_play > i_max_play:
        hover_r_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        hover_h_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        playing = False
        hover_path_tool.callback = hover_pathCB
        curdoc().remove_periodic_callback(periodic_cb)
        time_path_pretext.text=init_text_time_path_pretext
def play_buttonCB(event):
    global i_min_play
    global i_max_play
    global i_play
    global periodic_cb
    global hover_path_tool
    global playing
    global playing_div

    if playing:
        hover_r_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        hover_h_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        if periodic_cb!=None:
            curdoc().remove_periodic_callback(periodic_cb)

    i_min_play=findIndexWithVal(path_H_source.data["stamp"], t_min_g, 0)
    i_max_play=findIndexWithVal(path_H_source.data["stamp"], t_max_g, 0)
    i_play = i_min_play

    playing = True
    hover_path_tool.callback = None
    playing_div.text=init_playing_div_text + "playing"
    periodic_cb = curdoc().add_periodic_callback(play_buttonCBp, 100)
    
    # real time : 1000ms <=> 120 index
play_button.on_click(play_buttonCB)

# Button pause
pause_button = Button(label="Pause", button_type="primary", width_policy="min", align="center")
def pause_buttonCB(event):
    global periodic_cb
    global time_path_pretext
    global playing_div
    if playing and periodic_cb!=None:
        curdoc().remove_periodic_callback(periodic_cb)
        playing_div.text=init_playing_div_text + "paused"
        periodic_cb = None
        time_path_pretext.text=init_text_time_path_pretext + "{:.1f}".format(path_H_source.data['stamp'][i_play]) + "s"
pause_button.on_click(pause_buttonCB)

# Button resume
resume_button = Button(label="Resume", button_type="primary", width_policy="min", align="center")
def resume_buttonCB(event):
    global periodic_cb
    global playing_div
    if playing and periodic_cb==None:
        playing_div.text=init_playing_div_text + "playing"
        periodic_cb = curdoc().add_periodic_callback(play_buttonCBp, 100)
resume_button.on_click(resume_buttonCB)

# Button stop
stop_play_button = Button(label="Stop", button_type="danger", width_policy="min", align="center")
def stop_play_buttonCB(event):
    global periodic_cb
    global hover_path_tool
    global playing
    global time_path_pretext
    global playing_div
    if playing:
        hover_r_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        hover_h_source.data = {'x':[], 'y':[], 'theta':[], 'stamp':[]}
        playing = False
        hover_path_tool.callback = hover_pathCB
        if periodic_cb!=None:
            curdoc().remove_periodic_callback(periodic_cb)
            periodic_cb = None
        time_path_pretext.text=init_text_time_path_pretext
        playing_div.text=init_playing_div_text + "stopped"
stop_play_button.on_click(stop_play_buttonCB)

# Pretext time paused
init_text_time_path_pretext = "time paused:"
time_path_pretext = PreText(text=init_text_time_path_pretext)

# Div playing
init_playing_div_text = "<b>Animation : </b>"
playing_div = Div(text=init_playing_div_text + "stopped")

# Button compute metrics
compute_metrics_button = Button(label="Compute", button_type="danger", width_policy="min", align="center")
def computeTotalTime():
    eps = 0.01
    start_found = False
    start_time = 0
    end_found=False
    end_time = 0
    last_not_null = False
    for i, time in enumerate(vel_h_time):
        if time < t_min_g:
            continue
        if time > t_max_g:
            break
        if start_found==False and vel_h_data[i] > 0:
            start_found = True
            start_time = time
            end_time = time
            last_not_null = True
        if last_not_null and vel_h_data[i]>-eps and vel_h_data[i]<eps :
            end_time=time
            end_found=True
        if vel_h_data[i]<-eps or vel_h_data[i]>eps:
            last_not_null=True
        else:
            last_not_null=False
    if end_found == False:
        print("\t-total_time not found")
    else:
        print("start_time={}".format(start_time))
        print("end_time={}".format(end_time))
        print("\t-total_time:\t{:.2f} start={:.2f}s end={:.2f}s".format(end_time-start_time, start_time, end_time))
def computeMinTTC():
    found = False
    ttc_min = ttc_data[0]
    ttc_min_time = ttc_time[0]
    for i, time in enumerate(ttc_time):
        if time < t_min_g:
            continue
        if time > t_max_g:
            break
        if ttc_data[i] < ttc_min:
            found = True
            ttc_min = ttc_data[i]
            ttc_min_time = time
    if found == False:
        print("\t-ttc_min not found")
    else:
        print("\t-ttc_min:\t{:.2f}, {:.2f}s".format(ttc_min, ttc_min_time))
def computeMinHRDist():
    found = False
    hrdist_min = dist_data[0]
    hrdist_min_time = dist_time[0]
    for i, time in enumerate(dist_time):
        if time < t_min_g:
            continue
        if time > t_max_g:
            break
        if dist_data[i] < hrdist_min:
            found = True
            hrdist_min = dist_data[i]
            hrdist_min_time = time
    if found == False:
        print("\t-hrdist_min not found")
        return False
    else:
        print("\t-hrdist_min:\t{:.2f}, {:.2f}s".format(hrdist_min, hrdist_min_time))
        return hrdist_min, hrdist_min_time
def getClosestDist(t):
    index_closest = -1
    for i, time in enumerate(dist_time):
        if time < t:
            continue
        else:
            index_closest = i
            if abs(dist_time[i-1]-t) < abs(time-t):
                index_closest-=1
    if index_closest == -1:
        print("ERREUR index_closest")
    else:
        return dist_data[index_closest]
def computeRVelAtMinDist(hrdist_min,hrdist_min_time):
    if hrdist_min_time == False:
        print("ERROR input")
        return False
    i_vel_r = -1
    for i, time in enumerate(vel_r_time):
        if time > hrdist_min_time:
            i_vel_r = i-1
            break
    if i_vel_r==-1:
        print("\t-velR_mindist not found")
    else:
        average_vel = 0
        for i in range(-2,3):
            average_vel+=vel_r_data[i_vel_r+i]
        average_vel/=5

        count = 1
        average_one_sec_vel = vel_r_data[i_vel_r]
        av_one_ratio = vel_r_data[i_vel_r]/getClosestDist(vel_r_time[i_vel_r])
        i = i_vel_r-1
        while vel_r_time[i_vel_r]-vel_r_time[i]<1:
            average_one_sec_vel+=vel_r_data[i]
            av_one_ratio+=vel_r_data[i]/getClosestDist(vel_r_time[i])
            count+=1
            i-=1
        average_one_sec_vel/=count
        
        print("\t-velR_mindist:\t{:.2f}, {:.2f}s".format(vel_r_data[i_vel_r], vel_r_time[i_vel_r]))
        print("\t-av_velR_min:\t{:.2f}".format(average_vel))
        print("\t-av_one:\t{:.2f}".format(average_one_sec_vel))
        print("\t-av_one_ratio:\t{:.2f}".format(av_one_ratio))
def compute_metrics_buttonCB(event):
    print("Metrics:")
    computeTotalTime()
    computeMinTTC()
    hrdist_min, hrdist_min_time = computeMinHRDist()
    computeRVelAtMinDist(hrdist_min, hrdist_min_time)

compute_metrics_button.on_click(compute_metrics_buttonCB)

# Div metrics
metrics_div_text_init = "Metrics:"
metrics_div = Div(text=metrics_div_text_init)

######################################################################################################
######################################################################################################
############
## LAYOUT ##
############

plot_size_column = column(plot_size_div, height_slider, width_slider, reset_plot_size_button)
legend_column = column(legend_div, show_legend_button, hide_mute_button_div, hide_mute_button)
other_column = column(other_div, reset_button, set_range_mvt_button)
t_range = row(column(t_min_input, row(t_min_minus_button, t_min_plus_button, align='center')), column(t_max_input, row(t_max_minus_button, t_max_plus_button, align='center')))
range_column = column(range_div, t_range, reset_range_button)
first_row_graph = row(plot_size_column, legend_column, range_column, other_column)
graph_column = column(first_row_graph, p1, p2, p3, p4)
anim_row = row(play_button, column(pause_button, resume_button, align="center"), stop_play_button, playing_div, time_path_pretext, align="start")

metrics_row = row(compute_metrics_button, metrics_div)

path_column = column(time_colored_path_div, p_path, anim_row, metrics_row)

layout = layout(
    [
        [inhus_div],
        [graph_column, path_column],
    ])

######################################################################################################
######################################################################################################
##########
## SHOW ##
##########


# curdoc().theme = 'dark_minimal'
curdoc().add_root(layout)
curdoc().title = "InHuS Logs"
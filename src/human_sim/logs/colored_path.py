import numpy as np
from PIL import Image, ImageDraw, ImageFont
import sys

def convert_pose_map(pose):
    nx=int((pose[1]+8)/0.025)
    ny=int((pose[0]+2)/0.025)
    return (nx,ny)
 
def compute_color(time): 
    r=0
    g=0
    b=0

    if time>=0 and time<0.17:
        r=255
        b=0
        g=1530*time
    elif time>= 0.17 and time<0.33:
        g=255
        b=0
        r=-1530*time+510
    elif time>= 0.33 and time<0.50:
        g=255
        r=0
        b=1530*time-510
    elif time>= 0.50 and time<0.67:
        b=255
        r=0
        g=-1530*time+1020
    elif time>= 0.67 and time<0.83:
        b=255
        g=0
        r=1530*time-1020
    elif time>= 0.83 and time<=1:
        r=255
        g=0
        b=-1530*time+1530

    return (int(r),int(g),int(b))

if len(sys.argv)>=2:
    filename =sys.argv[1]
f = open(filename, "r")

# start log line
string = f.readline()

path_H = []
path_R = []

for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")

        if mylist[2] == 'H':
            path_H.append((float(mylist[3]), (float(mylist[4]), float(mylist[5]))))

        elif mylist[2] == 'R':
            path_R.append((float(mylist[3]), (float(mylist[4]), float(mylist[5]))))
f.close()


im = Image.open("map.png").convert("RGBA")
draw = ImageDraw.Draw(im)

size = 4

show_H=True
show_R=True
if len(sys.argv)>=3:
    if sys.argv[2] == 'B':
        show_H=True
        show_R=True
    elif sys.argv[2] == 'H':
        show_H=True
        show_R=False
    elif sys.argv[2] == 'R':
        show_H=False
        show_R=True

# get time start time end
Td=0
Tf=max(path_H[-1][0], path_R[-1][0])
if len(sys.argv)>=4:
    Td=float(sys.argv[3])
if len(sys.argv)>=5:
    Tf=float(sys.argv[4])
dT = Tf-Td

# draw
# H
if show_H:
    for i in range(len(path_H)):
        if(path_H[i][0] >= Td and path_H[i][0] <= Tf):
            ratio = (path_H[i][0] - Td)/dT
            color = compute_color(ratio)
            pose = convert_pose_map(path_H[i][1])
            draw.rectangle((pose[0]-size, pose[1]-size, pose[0]+size, pose[1]+size), fill=color)

# R
if show_R:
    for i in range(len(path_R)):
        if(path_R[i][0] >= Td and path_R[i][0] <= Tf):
            ratio = (path_R[i][0] - Td)/dT
            color = compute_color(ratio)
            pose = convert_pose_map(path_R[i][1])
            draw.rectangle((pose[0]-size, pose[1]-size, pose[0]+size, pose[1]+size), fill=color)

# Legend
legend_pose = (30,20)
legend_size = 400
n=5
off_txt = (-15, 20)
for i in range(0,n+1):
    x = int(legend_pose[0] + i*legend_size/n)
    t = Td + i*dT/n
    out = "%.1f" % t
    draw.text((x+off_txt[0],legend_pose[1]+off_txt[1]), out, font=ImageFont.truetype("FreeMonoBold.ttf", 18), fill=(0,0,0))
for x in np.arange(legend_pose[0], legend_pose[0]+legend_size, 0.01):
    ratio = (x - legend_pose[0])/legend_size
    color = compute_color(ratio)
    draw.rectangle((int(x)-4, legend_pose[1]-8, int(x)+4, legend_pose[1]+8), fill=color)


im.show()

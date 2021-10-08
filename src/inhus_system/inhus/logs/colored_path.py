import numpy as np
from PIL import Image, ImageDraw, ImageFont
import sys

# def convert_pose_map(pose):
#     nx=int((pose[1]+10)/0.025)
#     ny=int((pose[0]+5)/0.025)
#     return (nx,ny)

def convert_pose_map(pose):
    nx=int((pose[0]+0.5)/0.025)
    ny=int(size_im[1]-(pose[1]+0.5)/0.025)
    return (nx,ny)

def compute_color(time):
    r=0
    g=0
    b=0

    if time>=0 and time<0.2:
        r=-1275*time+255
        g=255
        b=0
    elif time>= 0.2 and time<0.4:
        r=0
        g=255
        b=1275*time-255
    elif time>= 0.4 and time<0.6:
        r=0
        g=-1275*time+765
        b=255
    elif time>= 0.6 and time<0.8:
        r=1275*time-765
        g=0
        b=255
    elif time>= 0.8 and time<=1:
        r=255
        g=0
        b=-1275*time+1275

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


# im = Image.open("log_data/laas_adream.png").convert("RGBA")
im = Image.open("log_data/passage_hri.png").convert("RGBA")
# im = Image.open("log_data/corridor_hri.png").convert("RGBA")
# im = Image.open("log_data/narr_corridor_hri.png").convert("RGBA")
draw = ImageDraw.Draw(im)
size_im = (draw.im.size[0],draw.im.size[1])
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
Td=min(path_H[0][0], path_R[0][0])
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
legend_on = True
# legend_on = False
if legend_on:
    legend_pose = (30,20)
    legend_size = 400
    n=5
    off_txt = (-15, 20)
    draw.rectangle( (legend_pose[0]-20, legend_pose[1]-10, legend_pose[0]+450, legend_pose[1]+40), fill=(255,255,255))
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

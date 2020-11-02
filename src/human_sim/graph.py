import matplotlib.pyplot as plt
import sys

for arg in sys.argv:
    filename = arg

f = open(filename, "r")

# start
string = f.readline()

list_time = []
list_path = []

for line in f:
    if 'str' in line:
        break
    if line != "\n":
        line = line.replace("\n","")
        mylist = line.split(" ")
        mylist = mylist[3:5]

        time = float(mylist[0])
        path = int(mylist[1])

        list_time.append(time)
        list_path.append(path)


f.close()

plt.plot(list_time, list_path, 'b+')
plt.show()

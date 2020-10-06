import sys, socket, json

host = "localhost"

# Port of the 'stream' of the pose sensor. By default,
#starts at 60000 and increments for each sensor. You can have a look to
#MORSE console output to know exactly which port is used by which sensor.
port = 60000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))
morse = sock.makefile("r")

data = json.loads(morse.readline())

print(str(data))

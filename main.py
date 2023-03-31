import serial

import matplotlib.pyplot as plt
import numpy as np

arduino = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1.0)
line = arduino.readline().decode('utf-8').split(' ')
print(line)
plt.ion()
fig=plt.figure()


i=0
x=list()
y=list()
i=0
while True:

    data = arduino.readline().decode('utf-8').split(' ')
    print(data)
    if(data[2] != "anglex:"):
        continue
    print(data[3])
    x.append(i)
    y.append(data[3])

    plt.scatter(i, data[3])
    i += 1
    plt.show()
    plt.pause(0.1)  # Note this correction


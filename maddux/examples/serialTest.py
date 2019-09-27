import serial
import time
import numpy as np

test = ["111111111","222222222","333333333"]
ser = serial.Serial(port='COM9', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS, timeout=3.0)

time.sleep(2)
print "serial open"
# ser.write("000000000")

targetAngles = '000000000'
qTemp = np.array([0.1,0.1,0.1,0.1])
count = 0

while count < 200:
    # ser.write("020030020")
    targetAngles = str(int(qTemp[1])).zfill(3) + str(int(qTemp[2] + 90)).zfill(3) + str(int(qTemp[3] + 45)).zfill(3)
    print(targetAngles)
    # ser.write(targetAngles)
    qTemp = qTemp + 1
    count += 1
    # ser.write("000000000")
    # time.sleep(0.3)
    # data = ser.readline()[:-2]  # the last bit gets rid of the new-line chars
    # if data:
    #     print data




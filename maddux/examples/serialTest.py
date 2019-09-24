import serial
import time

test = ["111111111","222222222","333333333"]
ser = serial.Serial(port='COM3', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS, timeout=3.0)

time.sleep(2)
print "serial open"
ser.write("123456789\r\n")
print 'sent'


while True:
    ser.write("123456789\n")
    time.sleep(0.3)
    data = ser.readline()[:-2]  # the last bit gets rid of the new-line chars
    if data:
        print data




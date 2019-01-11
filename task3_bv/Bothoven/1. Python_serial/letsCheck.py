import serial
import time

ser =serial.Serial("COM5", 9600)
ser.write(b'a')
time.sleep(1)


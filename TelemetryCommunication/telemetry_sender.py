import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 57600)
while True:
    ser.write(b'Hello from Pi 1\n')
    time.sleep(1)


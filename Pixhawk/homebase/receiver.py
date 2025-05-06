import serial

ser = serial.Serial('/dev/ttyAMA0', 57600)
while True:
    print(ser.readline().decode('utf-8').strip())





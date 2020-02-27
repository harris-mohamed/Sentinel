import serial  

ser = serial.Serial('/dev/ttyACM0', 115200)

while True:
    read_serial = ser.readline()
    print(read_serial)

import serial
import time

with serial.Serial('/dev/serial0', 57600) as ser:
    while True:
        if ser.in_waiting > 0:
            print(ser.read())
        time.sleep(0.01)
        for i in range(10):
            ser.write('hello'.encode('utf-8'))
            time.sleep(1)

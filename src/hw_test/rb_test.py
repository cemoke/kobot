from smbus2 import SMBus
import time
import os

# Write a byte to address 80, offset 0
bus = SMBus(1)
while True:
    val_list = []
    begin = time.time()
    for i in range(8):
        val = bus.read_byte_data(96, i)
        val_list.append(val)
    print(time.time()-begin)
    time.sleep(0.05)
    print(val_list)


import numpy as np
from lx16a import *
import time

print("im gonna robo-pre")

COM = '/dev/ttyUSB0'

LX16A.initialize(COM)

servo_9 = LX16A(9)

servo_9.moveTimeWrite(50, 100)
time.sleep(1)
print(servo_9.getVirtualPos())

time.sleep(2)

servo_9.moveTimeWrite(100, 100)
time.sleep(1)
print(servo_9.getVirtualPos())

print(servo_9.IDRead())
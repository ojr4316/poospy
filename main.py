import numpy as np
from lx16a import *
import time
from IK_3DOF import *
from IK_gyration import *
from activity import *
from hexapod import *

def establish_serial(osString='/dev/ttyUSB'):
    connected = False
    for i in range(4):
        if not connected:
            try:
                COM = osString + str(i)
                print("Trying to establish connection with " + COM)
                LX16A.initialize(COM)
                connected = True
                print("Successfully connected to " + COM)
            except:
                print('Failed!')
                
def build_hexapod():
    servo_1 = LX16A(1)
    servo_2 = LX16A(2)
    servo_3 = LX16A(3)
    servo_4 = LX16A(4)
    servo_5 = LX16A(5)
    servo_6 = LX16A(6)
    servo_7 = LX16A(7)
    servo_8 = LX16A(8)
    servo_9 = LX16A(9)
    servo_10 = LX16A(10)
    servo_11 = LX16A(11)
    servo_12 = LX16A(12)
    servo_13 = LX16A(13)
    servo_14 = LX16A(14)#
    servo_15 = LX16A(15)
    servo_16 = LX16A(16)
    servo_17 = LX16A(17)
    servo_18 = LX16A(18)

    initial = [0, 0, 0]
    leg_1 = Leg(initial, [servo_1, servo_2, servo_3], 1)
    leg_2 = Leg(initial, [servo_4, servo_5, servo_6], 2)
    leg_3 = Leg(initial, [servo_7, servo_8, servo_9], 3)
    leg_4 = Leg(initial, [servo_10, servo_11, servo_12], 4)
    leg_5 = Leg(initial, [servo_13, servo_14, servo_15], 5)
    leg_6 = Leg(initial, [servo_16, servo_17, servo_18], 6)
    
    return Hexapod([leg_1, leg_2, leg_3, leg_4, leg_5, leg_6])
    
establish_serial()
robot = build_hexapod()

home = gyration(np.array([[0], [0], [65.82]]), np.array([[0], [0], [0]]))
print(home)

robot.move(home)

# --------- Structure of Hexapod --------------------#
#            ____
#          1      2
#        /          \
#       3            4
#        \          /
#          5 ____ 6
#
#
#




# All lengths are in mm
import numpy as np
from hexapod.leg import Leg
from motion.maths import *


#----------------------------------------------------#
# Utilizing Wave Gait                                #
#----------------------------------------------------#





class Hexapod:

#-------------- CONSTANTS ---------------------------#

    #Each leg has 3 angles (COXIA, FEMUR, TIBIA)
    coxia = 65 / 1000
    femur = 40 / 1000
    tibia = 140 / 1000

    # Max velocity in (m/sec)
    max_vel = 0.05

    # L3 would equal the height of the robot
    height = 200 / 1000

    # Duty factor of a wave gait
    beta = 0.75

    # Vector from the center to the hip
    hip_vector = 185 / 1000

    # Stride length of each leg
    stride_length = 50 / 1000

    # Max height of the leg for the transfer phase
    leg_height_max = 50 / 1000


    def __init__(self, legs = None):
        self.origin_vector = Vector(0, 0, self.height)
        self.orientation = [0,0,0]
        self.legs = legs
    
    def update_orientation(self, r):
        self.orientation = r

    def update_origin(self, o):
        self.origin_vector = o

# 6 legs 6 points

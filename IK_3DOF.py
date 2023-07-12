#Inverse Kinematic Controler for Single Leg 3DOF Movement 
#Origin is hip joint of robot 

import numpy as np
from array import *

#INPUTS, XYZ coords
ut_old = np.array([[163.14],
               [0],
               [-91.2]])

#length of each leg segment (mm)
seg1 = 65
seg2 = 40
seg3 = 135

def ik3dof(ut):
    #calculating position of second joint (assume level robot)
    alpha = np.arctan(ut[1][0]/ut[0][0]) #radians
    joint2 = np.array([[seg1 * np.cos(alpha)],
                       [seg1 * np.sin(alpha)],
                       [0]])

    joint2_to_ee = joint2 - ut
    air_side = np.sqrt(joint2_to_ee[0][0]**2 + joint2_to_ee[1][0]**2 + joint2_to_ee[2][0]**2)
    mod = np.arccos(np.sqrt(joint2_to_ee[0][0]**2 + joint2_to_ee[1][0]**2)/air_side) * 180/np.pi
    beta = np.arccos((seg2**2 + air_side**2 - seg3**2)/(2 * seg2 * air_side)) * 180/np.pi - mod
    gamma = (np.pi - np.arccos((seg2**2 + seg3**2 - air_side**2)/(2 * seg2 * seg3))) * 180/np.pi
    
    return [alpha, beta, gamma]

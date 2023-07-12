
import numpy as np
from leg import Leg
import time

class Hexapod:
#-------------- CONSTANTS ---------------------------#
    
    # All lengths are in mm
    #radius of body (from center to hip)
    body_radius = 100 #mm

    #angle between legs
    leg_spread = 60 * np.pi/180 #degree input converted to radians 

    #length of each leg segment (mm)
    seg1 = 100
    seg2 = 220
    seg3 = 296
    
    def __init__(self, legs = None):
        self.orientation = [0,0,0]
        self.legs = legs
    
    def move(self, pos, t=500):
        alphas = pos[0]
        betas = pos[1]
        gammas = pos[2]
        
        for leg in self.legs:
            leg.move_alpha(alphas[leg.id-1], t)
            leg.move_beta(-betas[leg.id-1], t)
            leg.move_gamma(gammas[leg.id-1], t)

        

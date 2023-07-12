import threading
import numpy as np
import time

def linear_interp(servo, pos):
    init = servo.getVirtualPos()
    duration = 5
    num_steps = 100
    sleep_time = duration / num_steps
    for i in range(num_steps):
        t = i / num_steps
        cpos = init + (pos - init) * t
        print("moving " + str(i) + ": " + str(cpos))
        servo.moveTimeWrite(int(cpos))
        time.sleep(sleep_time)

def interp(servo, pos=100):
    init = 0
    duration = 5
    num_steps = 100
    sleep_time = duration / num_steps
    
    def ease_in_out(t):
        return 0.5 - 0.5 * np.cos(np.pi * t)
    
    easing_values = ease_in_out(np.linspace(0, 1, num_steps, True))
    eased = ease_in_out(easing_values)
    print(eased)
    points = init + eased * (pos-init)
    print(points)
    
    #for i in range(num_steps):
        #t = i / num_steps
        #cpos = init + (pos - init) * t
        #print("moving " + str(i) + ": " + str(cpos))
        #servo.moveTimeWrite(int(cpos))
        #time.sleep(sleep_time)


class Activity:
    
    def start(self):
        if not self.thread.is_alive():
            self.thread.start()
    
    def cancel(self):
        if self.thread.is_alive():
            self.thread.cancel()
    
    def __init__(self, time, action, trigger=True):
        self.time = time
        self.action = action
        self.thread = threading.Timer(time, action)
        if trigger:
            self.start()
        
        
class MotorSmooth:
    def __init__(self, alpha):
        self.alpha = alpha
        self.previous_command = 0.0
    
    def smooth(self, servo, pos):
        smoothed = self.alpha * pos + (1-self.alpha) * self.previous_command
        servo.moveTimeWrite(int(smoothed))
        self.previous_command = smoothed

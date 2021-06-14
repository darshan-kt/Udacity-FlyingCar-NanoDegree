# -*- coding: utf-8 -*-

'''
Moving into 2D drone
Drone has two propellers each located a distance 𝑙 from the center of mass. In this exercise, we will ignore the yaw-inducing reactive moment from each propeller.
The state can be described by the vector:𝑋=[𝑧,𝑦,𝜙,𝑧˙,𝑦˙,𝜙˙]
We will have to track the drone's position in 2 dimensions and its rotation about the 𝑥 axis, which is directed into the plane. 
'''

import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

class Drone2D:
    
    def __init__(self,
                 k_f = 0.1, # value of the thrust coefficient
                 i = 0.1,   # moment of inertia around the x-axis
                 m = 1.0,   # mass of the vehicle 
                 l = 0.15,  # distance between the center of mass and the propeller axis
                ):
        
        self.k_f = k_f
        self.i = i
        self.l = l 
        self.m = m
        
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81
        
        # z, y, phi, z_dot, y_dot, phi_dot
        self.X = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        
    def advance_state_uncontrolled(self,dt):
        """Advances the state of the drone by dt seconds. 
        Note that this method assumes zero rotational speed 
        for both propellers."""
        
        X_dot = np.array([
            self.X[3], 
            self.X[4], 
            self.X[5], 
            self.g, 
            0.0, 
            0.0])
        # Change in state will be 
        self.X = self.X + X_dot * dt
        return self.X
    

drone = Drone2D()
Z_history = []
Y_history = []
dt = 0.1

# add a slight initial horizontal velocity
drone.X[4] = 2.0          #horizontal velocity drone travels horizontally while coming down.     ********Tune this value for observing horizental motion in graph

for _ in range(100):
    Z_history.append(drone.X[0])               #1. Stores the first value as 0 and later based on drone.advance_state_uncontrolled() update Z value will add on
    Y_history.append(drone.X[1])
    
    # call the uncontrolled (free fall) advance state function
    drone.advance_state_uncontrolled(dt)       #2
    
plt.plot(Y_history, Z_history )             #ploting upward and horizontal values plot

# invert the vertical axis so down is positive 
plt.gca().invert_yaxis()
plt.xlabel("Horizontal Position (y)")
plt.ylabel("Vertical Position (z)")
plt.show()
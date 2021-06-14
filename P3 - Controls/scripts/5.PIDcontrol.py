# -*- coding: utf-8 -*-
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import mono_plotting
import trajectories


# Monorotor modelling

class Monorotor:
    def __init__(self, m=10):
        self.m = m
        self.g = 9.81
        
        # Note that we're no longer thinking of rotation rates. We are thinking directly in terms of thrust.
        self.thrust = 0.0
        
        #z, z_dot
        self.X = np.array([0.0, 0.0])
        
    @property
    def z(self):
        return self.X[0]
    
    @property
    def z_dot(self):
        return self.X[1]
    
    @property
    def z_dot_dot(self):
        f_net = self.m * self.g - self.thrust
        return f_net / self.m
    
    def advance_state(self, dt):
        X_dot =np.array([
            self.z_dot, 
            self.z_dot_dot])
        
        self.X = self.X + X_dot * dt
        # print(self.X)
        return self.X
    
'''
Note: we've introduced integrated_error and set it to 0.0 at initialization. 
We also now need to know how much time dt has elapsed between successive calls to thrust_control as this influences the integrated error.
'''

#PID controller
class PIDController:
    
    def __init__(self, k_p, k_d, k_i, m):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.vehicle_mass = m
        self.g = 9.81
        self.integrated_error = 0.0
        
    def thrust_control(self,
                z_target, 
                z_actual, 
                z_dot_target, 
                z_dot_actual,
                dt=0.1,
                z_dot_dot_ff=0.0):
        
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        self.integrated_error += err * dt
        
        p = self.k_p * err
        i = self.integrated_error * self.k_i
        d = self.k_d * err_dot
         
        u_bar = p + i + d + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar)
        return u
    
'''
Run the simulation code below with K_I = 0.0 to remind yourself what a PD controller does when there's a large mass_error (remember, a PID controller with zero integral gain is just a PD controller).

Then slowly increase K_I and rerun the cell. Note what happens to the error.
'''


#Refer the udacity given table for tuning PID terms wrt to overshoot, Trise, Tsettle
MASS_ERROR = 1.5
K_P = 18.0
K_D = 5.0        #Increase to reduce initial overshoot
K_I = 5      # TODO - increase to 0.5, 1, 2, etc...

z_dot_dot_ff = 2    #Tune this param to see the I term effectccccc(Initial target accn)

AMPLITUDE = 0.5
PERIOD    = 0.4

# preparation 
drone = Monorotor()
perceived_mass = drone.m * MASS_ERROR
controller    = PIDController(K_P, K_D, K_I, perceived_mass)

t, z_path, z_dot_path = trajectories.step(duration=10.0)

dt = t[1] - t[0]

# run simulation
history = []
for z_target, z_dot_target in zip(z_path, z_dot_path):
    z_actual = drone.z
    z_dot_actual = drone.z_dot

    u = controller.thrust_control(z_target, z_actual, 
                                  z_dot_target, z_dot_actual,
                                  dt, z_dot_dot_ff)
    
    drone.thrust = u
    drone.advance_state(dt)
    history.append(drone.X)
    
# generate plots
z_actual = [h[0] for h in history]
mono_plotting.compare_planned_to_actual(z_actual, z_path, t)   
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
    
    
#PD control with FeedForward
class PDController:
    
    def __init__(self, k_p, k_d, m):
        self.k_p = k_p
        self.k_d = k_d
        self.vehicle_mass = m
        self.g = 9.81
    
    def thrust_control(self,
                z_target, 
                z_actual, 
                z_dot_target, 
                z_dot_actual,
                z_dot_dot_ff=0.0):
        
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        u_bar = self.k_p * err + self.k_d * err_dot + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar) 
        
        return u
    
'''
Compare trajectories with and without a feedforward term
The code below generates plots of 𝑧 vs 𝑡 for two drones. One uses FF and the other doesn't.
Run the code and compare the two trajectories. What happens if you increase the oscillation frequency to 10? What happens if you decrease it to 2?
You should notice a lag in the system response: the trajectory without the feedforward term should lag behind the desired trajectory in time. This effect diminishes as the oscillation frequency decreases.
'''


# This code simulates TWO drones. One uses the feed forward
# acceleration and the other doesn't. Note the difference in
# trajectories.

MASS_ERROR = 1.0
K_P = 20.0
K_D = 10.0

AMPLITUDE = 0.5
OSCILLATION_FREQUENCY = 5

PERIOD = 2 * np.pi / OSCILLATION_FREQUENCY

# preparation (TWO drones to compare)
drone = Monorotor()
ff_drone = Monorotor()
perceived_mass = drone.m * MASS_ERROR

# instantiate TWO controllers
controller    = PDController(K_P, K_D, perceived_mass)
ff_controller = PDController(K_P, K_D, perceived_mass)

# get trajectories
t, z_path, z_dot_path, z_dot_dot_path = trajectories.cosine(AMPLITUDE, 
                                                            PERIOD,
                                                           duration=6.0)
dt = t[1] - t[0]
# run simulation
history = []
ff_history = []
for z_target, z_dot_target, z_dot_dot_ff in zip(z_path, 
                                                z_dot_path, 
                                                z_dot_dot_path):
    z_actual = drone.z
    z_dot_actual = drone.z_dot
    
    ff_z_actual = ff_drone.z
    ff_z_dot_actual = ff_drone.z_dot
    
    u_ff = controller.thrust_control(z_target, ff_z_actual, 
                                  z_dot_target, ff_z_dot_actual,
                                  z_dot_dot_ff)
    u = controller.thrust_control(z_target, z_actual, 
                                  z_dot_target, z_dot_actual)
    
    drone.thrust = u
    ff_drone.thrust = u_ff
    
    drone.advance_state(dt)
    ff_drone.advance_state(dt)
    
    history.append(drone.X)
    ff_history.append(ff_drone.X)
    
# generate plots
z_actual = [h[0] for h in history]
z_ff_actual = [h[0] for h in ff_history]
mono_plotting.compare_planned_to_actual(z_actual, z_path, t, 
                                   z_ff_actual)   
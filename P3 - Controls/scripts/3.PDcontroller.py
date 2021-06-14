import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import mono_plotting


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
    
    
#PD controller
class PDController:
    def __init__(self, k_p, k_d, m):
        self.k_p = k_p
        self.k_d = k_d
        self.vehicle_mass = m
        self.g = 9.81
        
        
    def thrust_control(self, z_target, z_actual, z_dot_target, z_dot_actual, z_dot_dot_ff=0.0):
        
        #error in position
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        
        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * err + self.k_d * err_dot
        
        u = self.vehicle_mass * (self.g - u_bar)
        
        return u
        
       
'''
 Param adjustments
 Start by running the code below with K_D = 0.0. This will remind you what a P-controlled trajectory looks like. Then try slowly increasing K_D to 0.5, 1.0, 2.0, etc...

What value of K_D gives a reasonable trajectory?

Is there a problem with setting K_D too high? 
 '''
       
        
MASS_ERROR = 1.5
K_P = 20.0
K_D = 4.0

# preparation
drone = Monorotor()
perceived_mass = drone.m * MASS_ERROR
controller = PDController(K_P, K_D, perceived_mass)

# generate trajectory
total_time = 3.0
dt = 0.001
t=np.linspace(0.0,total_time,int(total_time/dt))
z_path= -np.ones(t.shape[0])
z_dot_path = np.zeros(t.shape[0])

# run simulation
history = []
for z_target, z_dot_target in zip(z_path, z_dot_path):
    z_actual = drone.z
    z_dot_actual = drone.z_dot
    u = controller.thrust_control(z_target, z_actual, 
                                  z_dot_target, z_dot_actual)
    drone.thrust = u
    drone.advance_state(dt)
    history.append(drone.X)
    
# generate plots
z_actual = [h[0] for h in history]
mono_plotting.compare_planned_to_actual(z_actual, z_path, t)    
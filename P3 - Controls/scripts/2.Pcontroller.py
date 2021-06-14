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
    
    
# P Controller
class PController:
    
    def __init__(self, k_p, m):
        self.k_p = k_p
        self.vehicle_mass = m
        self.g = 9.81
        
    def thrust_control(self, z_target, z_actual):
        # error formuala/calculation
        err = z_target - z_actual
        
        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * err
        
        # u is the thrust commande which will cause u_bar
        u = self.vehicle_mass * (self.g - u_bar)
        
        return u
   
   
    
# Exploring P controller trajectories
MASS_ERROR = 1.0
K_P = 0.9

# preparation
drone = Monorotor()
perceived_mass = drone.m * MASS_ERROR
controller = PController(K_P, perceived_mass)

# generate trajectory
total_time = 10.0
dt = 0.001
t=np.linspace(0.0,total_time,int(total_time/dt))
z_path= -np.ones(t.shape[0])

# run simulation
history = []
for z_target in z_path:
    z_actual = drone.z
    print("z_actual", z_actual, "z_target", z_target)
    u = controller.thrust_control(z_target, z_actual)
    drone.thrust = u
    drone.advance_state(dt)
    history.append(drone.X)
    
# generate plots
z_actual = [h[0] for h in history]
# print("z_actual", z_actual)
mono_plotting.compare_planned_to_actual(z_actual, z_path, t)




'''
TODO 2 - Explore p controller trajectories

Once you see "Tests pass", your PController should be working! The code below attempts to fly the vehicle up to 𝑧=−1
from its start position at 𝑧=0 (this is also called a "step function" change in position).

Now, try playing around with the code below. Answer the following questions for yourself:

    What do "p controlled" trajectories generally look like (when trying to follow a step function change in position)?

    How do changes to K_P influence the resulting trajectory?

    How robust is this controller to changes in MASS_ERROR? Try changing the value to 1.5 and observing the result. Compare this to the open loop plot when this parameter was just 1.01.
'''
# -*- coding: utf-8 -*-
'''
Monorotor Review
we will design a controller for a monorotor that moves only in the vertical direction.
We want to focus on controls concepts, so we will be hiding or ignoring some of the details from the last lesson. Most importantly:

    1. Ignoring yaw - this will let us focus on 𝑧 motion.
    2. Ignoring propellers - we will set the thrust property directly instead of worrying about the rotational rates that generate that thrust.
'''

import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import mono_plotting

#Monorotor class (Drone model)

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
        print(self.X)
        return self.X
    
    '''
    Open loop controller
    This code implements an "open loop controller" for a monorotor.

    Notice that the controller only needs to know the mass of the vehicle and the initial state (assumed to be 𝑧= 0 and 𝑧˙=0).
    The thrust_control function is then called at each timestep. All that this function needs to know is where the drone should be (target_z) and when it needs to be there (dt)!
    '''

# OpenLoop Controller class.(Controller model) Read through thrust_control method to make sure to understand how it works.
class OpenLoopController:
    def __init__(self, vehicle_mass, initial_state = np.array([0.0])):
        self.vehicle_mass = vehicle_mass
        
        self.vehicle_state = initial_state
        self.g = 9.81
        
        
    def thrust_control(self, target_z, dt):
        """
        Returns a thrust which will be commanded to 
        the vehicle. This thrust should cause the vehicle
        to be at target_z in dt seconds.
        
        The controller's internal model of the vehicle_state
        is also updated in this method.
        """
        
        #1. find target velocity needed to get to target_z
        
        #Read vehicle state(z and z_dot)
        current_z, current_z_dot = self.vehicle_state
        # Find error in z
        delta_z = target_z - current_z
        # Find target velocity need to achieve error
        target_z_dot = delta_z / dt
        
        
        #2. find the target acceleration needed
        delta_z_dot = target_z_dot - current_z_dot
        target_z_dot_dot = delta_z_dot / dt
        
        #3. find target NET force
        target_f_net = target_z_dot_dot * self.vehicle_mass
        
        #4. find target thrust.  F_net = mg(down) - thrust(up)
        thrust = self.vehicle_mass * self.g - target_f_net
        
        #5. Update the controller intial state
        self.vehicle_state += np.array([delta_z, delta_z_dot])
        
        print("thrust", thrust)
        return thrust
    
    
        
'''
Simulating Open Loop Control

We will try to follow the following trajectory for 5.0 seconds:

        𝑧(𝑡)=0.5cos(2𝑡)−0.5
'''

# Generate and visualize the target trajectory

total_time = 5.0
t = np.linspace(0.0,total_time,1000)
dt = t[1] - t[0]
z_path= 0.5*np.cos(2*t)-0.5

plt.figure(figsize=(5,5))
plt.ylabel("z (meters)")
plt.xlabel("time (seconds)")
plt.gca().invert_yaxis()
plt.plot(t,z_path)
plt.show()



'''
 Read through simulation code¶

Read through sections 1 and 2 of the code below.

In section 1, note how perceived_mass is calculated and used.

In section 2, note how the simulation actually works:

    1. The controller commands a thrust based on the target_z.
    2. This drone's thrust is set to this value.

'''

# 1. Preparation for simulation

MASS_ERROR = 1.01             #keep it as 1, it will generate error less path
drone = Monorotor()
drone_start_state = drone.X
drone_mass = drone.m

# The mass that the controller believes is not necessarily the
# true mass of the drone. This reflects the real world more accurately.
perceived_mass = drone_mass * MASS_ERROR
controller = OpenLoopController(perceived_mass, drone_start_state)

# 2. Run the simulation
drone_state_history = []
for target_z in z_path:
    drone_state_history.append(drone.X)
    thrust = controller.thrust_control(target_z, dt)
    drone.thrust = thrust
    drone.advance_state(dt)

# 3. Generate plots
z_actual = [h[0] for h in drone_state_history] 
mono_plotting.compare_planned_to_actual(z_actual, z_path, t)


'''
 Modify MASS_ERROR

Play around with different values for this parameter. Try 1.0, 1.01, etc...
Note the effect this has on the resulting trajectory!
'''
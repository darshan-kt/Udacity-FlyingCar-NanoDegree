import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

# This script provides basic overview of state varibles and second order system. It is just like poc, advace implentation will be done in comming exercise
# In this script directly focus on advace_state_uncontroller() method
'''
What is state tracking?
State tracking is nothing but tracking the vehicle state vector variables.
X = (z, psi, z_dot, psi_dot) for 1D system
For example for 1D system(z-axis): state varibles are z(position), z_dot(vel) for linnear and yaw, yaw_rate
                                   To update above values need to track delta_z(change in position), delta_z_dot(CiV)
                                   
    As we know when drone always effected to gravity ie, 9.81m/s^2 accn is applying on drone.
    By using acceleration we can get velocity and positionn of drone easily by performing second order system method(ie double integration method)
    ie, When we integrate 1st time with accn we can get velocity.                                   delta_z_dot = z_dot_dot * dt
        When we integrate 2nd time/ integrate 1st time to previous velocity we can get position.    delta_z = delta_z_dot * dt    
                                  
'''


class CoaxialCopter:
    
    def __init__(self, 
                 k_f = 0.1,    # value of the thrust coefficient
                 k_m = 0.1,    # value of the angular torque coefficient
                 m = 0.5,      # mass of the vehicle 
                 i_z = 0.2,    # moment of inertia around the z-axis
                ):
        
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.i_z = i_z
        
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81
        
        
        #Declaring state varible for tracking   
        self.z = 0
        self.z_dot = 0
        self.psi = 0      #yaw
        self.psi_dot = 0
        
    
    @property
    def z_dot_dot(self):
        """Calculates current vertical acceleration."""
        
        f_1 = self.k_f * self.omega_1**2
        f_2 = self.k_f * self.omega_2**2
        f_g = self.m * self.g
        f_total = -f_1 - f_2 + f_g

        acceleration = f_total / self.m 

        return acceleration
    
    @property
    def psi_dot_dot(self): 
        """Calculates current rotational acceleration.""" 

        cw_torque = self.k_m * self.omega_1 **2
        ccw_torque = self.k_m * self.omega_2 **2

        net_torque = cw_torque - ccw_torque
        angular_acc = net_torque / self.i_z

        return angular_acc
    
    def set_rotors_angular_velocities(self, linear_acc, angular_acc): 
        """
        Sets the turn rates for the rotors so that the drone
        achieves the desired linear_acc and angular_acc.
        """

        term_1 = self.m * (-linear_acc + self.g) /(2 * self.k_f)
        term_2 = self.i_z * angular_acc/(2 * self.k_m)

        omega_1 = math.sqrt(term_1 - term_2)
        omega_2 = math.sqrt(term_1 + term_2)

        self.omega_1 = -omega_1
        self.omega_2 = omega_2

        return self.omega_1, self.omega_2
    
    
    #Function for state tracking
    def advace_state_uncontroller(self, dt):
        #Assuming only accn due to gravity acting on drone / find the accn of system at that instant using net_force method(This will implenment in future)
        z_dot_dot = 9.81
        
        # Get velocity / update velocity
        delta_z_dot = z_dot_dot * dt
        #Assining to global drone vel variable
        self.z_dot = self.z_dot + delta_z_dot
        
        # Get position / update position
        delta_z = self.z_dot * dt
        self.z = self.z + delta_z
        
        
        # As similar to linear accn perofrm for yaw and yaw_rate calculations 
        psi_dot_dot = 0.0
        
        delta_psi_dot = dt * psi_dot_dot
        self.psi_dot += delta_psi_dot
        
        # technically we should restrict psi so it's between 
        # -pi and pi, but we're not going to worry about that now
        delta_psi = self.psi_dot * dt
        self.psi += delta_psi
        


#For plotting
t = np.linspace(0.0, 2.0, 21)
dt = 0.1

drone = CoaxialCopter()

z_history = []
for i in range(t.shape[0]):
    z_history.append(drone.z)
    drone.advace_state_uncontroller(dt)

plt.plot(t,z_history)
plt.ylabel("Z position (meters)")
plt.xlabel("Time (seconds)")
plt.gca().invert_yaxis()
plt.show()
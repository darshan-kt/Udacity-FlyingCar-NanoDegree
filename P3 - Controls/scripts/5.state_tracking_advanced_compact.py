import numpy as np 
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

# State variables for declared in array for saving data and easy calculations



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
        
        
        #Declaring state varible for tracking   z, psi, z_dot, psi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0])

        
    
    @property
    def z_dot_dot(self):                 #4. This uses w1 and w2 returns acceleration(z_dot_dot)
        """Calculates current vertical acceleration."""
        
        f_1 = self.k_f * self.omega_1**2
        f_2 = self.k_f * self.omega_2**2
        f_g = self.m * self.g
        f_total = -f_1 - f_2 + f_g

        acceleration = f_total / self.m 

        # print("accn", acceleration)
        return acceleration
    
    @property
    def psi_dot_dot(self):               #5. This uses w1 and w2 returns angular acceleration(psi_dot_dot)
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

        return self.omega_1, self.omega_2             #3. Return w1 and w2 and this will assign to global w1,w2
    
    
    #Function for state tracking
    def advance_state(self,dt):
        """Advances the state of the drone forward by dt seconds"""
        X_dot = np.array([
            self.X[2],   #z_dot
            self.X[3],   #psi_dot
            self.z_dot_dot,
            self.psi_dot_dot
        ])
    
        delta_x = X_dot * dt
        self.X = self.X + delta_X
        
        '''
        This is compact and effective approach
        current_state = [z, z_dot]  (position, velocity)
        updated_state = [z_dot, z_dot_dot]    (velocity, accn)    Here velocity z_dot take from current global velocity and accn is taken from calculation of accn of drone at that instant
        
        As per theory, 
        we can get the position(Changed at that instant) by integrating velocity w.r.t dt(At that instant)   delta_z = z_dot * dt
        Update_Current_Position current.z = current.z + delta_z
        
        we can get the velocity(Changed at that instant) by integrating accn w.r.t dt(At that instant)       delta_z_dot = z_dot_dot * dt
        Update_Current_velocity current.z_dot = current.z_dot + delta_z_dot
        
        We can represt this in below form
        [z, z_dot] = [z, z_dot] + [delta_z_dot, delta_z_dot_dot] * dt
        
        self.X = [z, z_dot],            
        X_dot = [z_dot, z_dot_dot],  
        delta_x = X_dot * dt    
           
        self.X(Updated_current_state) =  self.X(Updated_current_state) + delta_x (Changed state at that instant)           
        '''
    



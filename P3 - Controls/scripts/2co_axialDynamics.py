
import numpy as np 
import math

'''
This same script can be used for quadrotor to control drone acceleartions and rotation 
by using the rotor physics equations and drone gravity.
Currently this script was implemented for 2rotors drone, it can be extended to quadrotors also
on that time need to add f3 and f4 forces, t3 and t4 torques and finally w3 and w4.
'''

class CoaxialCopter:
    
    def __init__(self, k_f=0.1, k_m=0.1, m=0.5, i_z=0.2):
        
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.i_z = i_z
        
        # Intial rotors angular rate
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81
      
     '''
      Advantage of usifn @property decorator is we can call this MF as just like varible. 
      Ex: a = CoaxialCopter()            Created object for class
      a.z_dot_dot()                      This is how generally call the MF of class using object
      a.z_dot_dot                        This is the u can call MF if u used @property. ie, here ur calling like DM not as MF still it return z_dot_dot() return value
     ''' 
        
    @property                     
    def z_dot_dot(self):
        """Calculations for current vertical accelerations"""
        
        #Lift force(thrust) for rotor 1
        f_1 = self.k_f * self.omega_1
        #Lift force for rotor 2
        f_2 = self.k_f * self.omega_2
        #Down force
        f_g = self.m * self.g
        
        #Net force
        f_total = -f_1 - f_2 + f_g      #rotor thrust for rep as -ve and down force rep as +ve
        
        #Linear acceleration 
        acceleration = f_total / self.m         #Force acting on drone F = m*a
        
        return acceleration
    
    @property
    def psi_dot_dot(self):
        '''Calculations for current rotational accelerations'''
        
        # Angular moment/torque for rotor 1 & 2
        cw_torque = self.k_m * self.omega_1 ** 2
        ccw_torque = self.k_m * self.omega_2 ** 2
        
        net_torque = cw_torque - ccw_torque
        
        #Angular acceleration
        angular_acc = net_torque / self.i_z    #Angualar moment/torque action on drone T = I*a
        
        return angular_acc
    
    def set_rotors_angular_velocity(self, linear_acc, angular_acc):
        """
        Sets the turn rates for the rotors so that the drone
        achieves the desired/target linear_acc and angular_acc.
        """
        
        # Calculations for omega_1 and omega_2 using derived equations
        term_1 = self.m * (-linear_acc + self.g) /(2 * self.k_f)
        term_2 = self.i_z * angular_acc/(2 * self.k_m)

        omega_1 = math.sqrt(term_1 - term_2)
        omega_2 = math.sqrt(term_1 + term_2)
        
        # Assign the omega_1 and omega_2 values to global omega values 
        self.omega_1 = -omega_1
        self.omega_2 = omega_2
        
        return self.omega_1, self.omega_2      # returning updated angular rates for motors to achieve desired Fth and ang_accn
    

#Testing
bi = CoaxialCopter()
#What is the rotor angular rates for achieve hover condition (ie, zero linnear_acceleration and zeor angular_accn)
stable_omega_1, stable_omega_2 = bi.set_rotors_angular_velocity(0.0, 0.0)
print('Drone achieves stable hover with angular velocity of %5.2f' % stable_omega_1, 
      'for the first propeller and %5.2f' % stable_omega_2, 
      'for the second propeller.')

#What is the rotor angular rates for achieve drone upward accelration with 2m/s^2
stable_omega_1, stable_omega_2 = bi.set_rotors_angular_velocity(2.0, 0.0)
print('Drone achieves stable hover with angular velocity of %5.2f' % stable_omega_1, 
      'for the first propeller and %5.2f' % stable_omega_2, 
      'for the second propeller.')

#What is the rotor angular rates for achieve drone upward accelration with 2m/s^2 and angular acceleration 4rad/sec
stable_omega_1, stable_omega_2 = bi.set_rotors_angular_velocity(2.0, 4.0)
print('Drone achieves stable hover with angular velocity of %5.2f' % stable_omega_1, 
      'for the first propeller and %5.2f' % stable_omega_2, 
      'for the second propeller.')
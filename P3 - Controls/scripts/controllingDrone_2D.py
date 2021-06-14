import numpy as np 
import math
import  matplotlib.pyplot as plt
import matplotlib.pylab as pylab


class Drone2D:
    def __init__(self, 
                 k_f = 0.1,   # value of the thrust coefficient
                 I_x = 0.1,   # moment of inertia around the x-axis
                 m = 1.0,     # mass of the vehicle 
                 l = 0.5,     # distance between the center of mass and the propeller axis
                 
                 ):
        
        self.k_f = k_f
        self.I_x = I_x
        self.l = l
        self.m = m
        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81
        
        #State variables. z, y, phi, z_dot, y_dot, phi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        
        
    def get_thrust_and_moment(self):
        ''' Helper function which calculates and returns the collective thrust and moment about the X axis '''
        f_1 = self.k_f * self.omega_1 ** 2
        f_2 = self.k_f * self.omega_2 ** 2
        
        # c is often used to indicate "collectice thrust" 
        c = f_1 + f_2
        M_x = (f_1 - f_2) * self.l
        return c, M_x   
        
    @property
    def z_dot_dot(self):                   #5
        '''Calculates vertical(z) acceleration of drone.'''
        c, M_x = self.get_thrust_and_moment()
        
        phi = self.X[2]    #getting phi(horizental rotation) from global state vector
        #Formula for vertical accn
        a_z = self.g - c * math.cos(phi) / self.m
        return a_z
    
    @property
    def y_dot_dot(self):                     #6
        ''' Calculates lateral(y) acceleration of drone'''
        c, M_x = self.get_thrust_and_moment()
        
        phi = self.X[2]    #getting phi(horizental rotation) from global state vector
        # print(phi)
        #Formula for lateral accn
        a_y = c * math.sin(phi) / self.m
        return a_y
    
    @property
    def phi_dot_dot(self):                   #7
        c, M_x = self.get_thrust_and_moment()
        
        #Formula for angular horigental accn
        angular_acc = M_x / self.I_x
        return angular_acc
    
    
    # Advance state calculations
    def advance_state(self, dt):                  #4
        '''Advances the state of the drone forward by dt seconds.'''
        X_dot = np.array([
            self.X[3],
            self.X[4],
            self.X[5],
            self.z_dot_dot,                      #5
            self.y_dot_dot,                      #6
            self.phi_dot_dot                     #7
        ])
        
        # Change in state will be
        delta_x = X_dot * dt
        self.X = self.X + delta_x
        # print(self.X)
        return self.X
    
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
        
    
    def set_rotors_angular_velocities(self,linear_acc):
        """
        Objective: Need to track desired linner_accn without considering horizental angle(phi)[This horigental angle directly given inside decomposition thrust w.r.t bodyFrame]
        Sets self.omega_1 and self.omega_2 to realize the desired linear_acc. (Means w1^2 and w2^2 combinely need to achieve linear accn)
        Note that this is for vertical motion ONLY(So there is no need to varry w^2 values of rotor to perform pitching or rotational acceleration, so w^2 + w^2 = 2w^2 ). 
        It is assumed that rotational acceleration and phi is zero. (Because here just tracking lin accn, rotation of bodyFrame/horizental_rotaion(phi) can already directly integrated with decomposing thrusts)
        
        Deriving omega,
        F = kf * w^2
        w^2 = F/kf
        F = m*a
        Fnet = Fdown - Fup
             = m * g - m * linner_accn
          F  = m(g - linner_accn)
          
        Therefore w^2 = m(g - linner_accn)/kf
        Considering 2 motors tracking vertical motion without any difference in motors speed, w1^2 = w2^2 and w^2 = w1^2 + w2^2 = 2w^2 
        Finally w = sqrt(m(g - linner_accn)/2*kf)
        """
        
        omega = math.sqrt(self.m * (-linear_acc + self.g) /(2 * self.k_f))

        self.omega_1 = omega
        self.omega_2 = omega

        return self.omega_1, self.omega_2            #2
    
    
#Testing advance_state and set_rotors
# Start by generating a target trajectory and target vertical acceleration
total_time = 3.0
dt = 0.002
t = np.linspace(0.0, total_time, int(total_time/dt))

z_path= 0.5*np.cos(2*t)-0.5             # Trajectory path below one is target accn to track this trajectory
z_dot_dot_path= -2.0*np.cos(2*t)        # Based on this target vertical accn, motor generates the omega and state variables updates drone position using advanceState()


# Try to follow the trajectory. 
# Store the state history as we go.

drone = Drone2D()
drone_state_history = drone.X
for i in range(t.shape[0]-1):
    
    # setting the propeller velocities 
    drone.set_rotors_angular_velocities(z_dot_dot_path[i])            #1
    
    # calculating the new state vector 
    drone_state = drone.advance_state(dt)                             #3
    
    # generate a history of vertical positions for drone and arrange using vstack
    drone_state_history = np.vstack((drone_state_history, drone_state))            #8
    
    

#Ploting
plt.plot(t,z_path,linestyle='-',marker='o',color='red')
plt.plot(t,drone_state_history[:,0],linestyle='-',color='blue')
plt.grid()
plt.title('Change in height').set_fontsize(20)
plt.xlabel('$t$ [sec]').set_fontsize(20)
plt.ylabel('$z-z_0$ [$m$]').set_fontsize(20)
plt.xticks(fontsize = 14)
plt.yticks(fontsize = 14)
plt.legend(['planned path','executed path'],fontsize = 18)
plt.show()
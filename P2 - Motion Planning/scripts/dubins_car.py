# -*- coding: utf-8 -*-

'''
The Dubin's Car is expressed with the following differential equations:

𝑥˙=𝑣∗𝑐𝑜𝑠(𝜃)
𝑦˙=𝑣∗𝑠𝑖𝑛(𝜃)
𝜃˙=𝑣∗𝑡𝑎𝑛(𝑢)

Where 𝑣 is the velocity (note in the previous video it was assumed that 𝑣=1) and 𝑢 is the steering angle. Both the velocity and steering angle are constrained inputs to mimic the physical world. 
For example, the steering angle may be limited a value in the range [−30,30] (degrees) and the velocity [0, 100] (km/hour). It's also not uncommon to set the velocity to be a constant value.
'''

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = 12, 12

#Defining the function which will return the steering angle randomly in the range of [−30,30]. Please feel free to play with the range.
# limit the steering angle range
STEERING_ANGLE_MAX = np.deg2rad(30)

def sample_steering_angle():
    return np.random.uniform(-STEERING_ANGLE_MAX, STEERING_ANGLE_MAX)

def simulate(state, angle, v, dt):
    # TODO: implement the dubin's car model
    # return the next state
    x = state[0]
    y = state[1]
    theta = state[2]    

    new_x = x + v*np.cos(theta)*dt #Original x + linear approximation for timestep dt
    new_y = y + v*np.sin(theta)*dt #Original y + linear approximation for timestep dt
    new_theta = theta + v*np.tan(angle)*dt #Original theta + linear approximation for timestep dt
    
    return [new_x, new_y, new_theta]

# feel free to play around with these
v = 5
dt = 0.1
total_time = 50

# initial state
states = [[0, 0, 0]]

for _ in np.arange(0, total_time, dt):
    angle = sample_steering_angle()
    state = simulate(states[-1], angle, v, dt)
    states.append(state)

states = np.array(states)

plt.plot(states[:, 0], states[:, 1], color='blue')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
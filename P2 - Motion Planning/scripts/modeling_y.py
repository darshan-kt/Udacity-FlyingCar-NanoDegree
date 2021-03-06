# -*- coding: utf-8 -*-
'''
In this notebook, you'll have an opportunity to play with ππ‘to see its effect on a simple model:

π¦Λ=βπ¦

Given a small enough ππ‘
value this model approximates π¦=πβπ‘ accurately.
'''

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12

# simulate() is the discrete version of the above differential equation.
def simulate(y, dt):
    return y - y * dt

#After the intial run, change the value of ππ‘ to get a feel for how its effect on the approximation. 
total_time = 10
# TODO: change the value of dt
dt = 0.1
timesteps = np.arange(0, total_time, dt)

# Initial value of y is 1
ys = [1]

for _ in timesteps:
    y = simulate(ys[-1], dt)
    ys.append(y)
    
plt.plot(timesteps, ys[:-1], color='blue')
plt.ylabel('Y')
plt.xlabel('Time')
plt.title('Dynamics Model')
plt.legend()
plt.show()
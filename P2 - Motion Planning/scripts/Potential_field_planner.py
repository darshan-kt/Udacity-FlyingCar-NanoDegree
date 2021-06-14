# -*- coding: utf-8 -*-

'''
Potential Field

In this notebook you'll create a potential field by implement and combine attractive and replusive forces. We can use the potential field to move in a direction closer to the goal (attraction) while avoiding obstacles (repulsion).
Recall from lecture and the attraction potential is:

𝐹𝑎𝑡𝑡=𝛼∗(𝑥−𝑥𝑔𝑜𝑎𝑙)

and repulsion potential, which is only computed when 𝑑(𝑥−𝑥𝑜𝑏𝑠)<𝑄𝑚𝑎𝑥:

𝐹𝑟𝑒𝑝=𝛽∗(1𝑄𝑚𝑎𝑥−1𝑑(𝑥−𝑥𝑜𝑏𝑠))∗1𝑑(𝑥−𝑥𝑜𝑏𝑠)2

where 𝑥𝑔𝑜𝑎𝑙 is the goal location, 𝑥𝑜𝑏𝑠 is the obstacle location and 𝑑(...) is the distance metric.
'''

import numpy as np 
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12

def attraction(position, goal, alpha):
    # TODO: implement attraction force
    return alpha * (np.array(position) - np.array(goal))

def repulsion(position, obstacle, beta, q_max):
    # TODO: implement replusion force
    position = np.array(position)
    obstacle = np.array(obstacle)
    return beta * ((1 / q_max) - (1 / np.linalg.norm(position - obstacle))) * (1 / np.linalg.norm(position - obstacle)**2)

# Below we'll generate the potential field. For the purposes of the visualization we'll compute the field for the entire environment.
# Generally you'll only want to compute the field within a range around the current position for use in local planning.

def potential_field(grid, goal, alpha, beta, q_max):
    x = []
    y = []
    fx = []
    fy = []
    
    obs_i, obs_j = np.where(grid == 1)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] == 0:
                
                # add attraction force
                force = attraction([i, j], goal, alpha)

                for (oi, oj) in zip(obs_i, obs_j):
                    if np.linalg.norm(np.array([i, j]) - np.array([oi, oj])) < q_max:
                        # add replusion force
                        force += repulsion([i, j], [oi, oj], beta, q_max)
                    
                x.append(i)
                y.append(j)
                fx.append(force[0])
                fy.append(force[1])

    return x, y, fx, fy

# generate environment
grid = np.zeros((30, 30))
grid[10:15,10:15] = 1.0
grid[17:25,10:17] = 1.0

goal  = [5, 5]

# constants
alpha = 1.0
beta = 2.0
q_max = 15

#Generating the potential field.
x, y, fx, fy = potential_field(grid, goal, alpha, beta, q_max)

#Plotting the field. 
plt.imshow(grid, cmap = 'Greys', origin='lower')
plt.plot(goal[1], goal[0], 'ro')
plt.quiver(y, x, fy, fx)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
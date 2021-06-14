#!/usr/bin/env python
# -*- coding: utf-8 -*- 

'''
Confguration Space
In this notebook you'll create a configuration space given a map of the world and setting a particular altitude for your drone. You'll read in a .csv file containing obstacle data which consists of six columns 𝑥, 𝑦, 𝑧 and 𝛿𝑥, 𝛿𝑦, 𝛿𝑧.
You can look at the .csv file. The first line gives the map center coordinates and the file is arranged such that:

𝑥-> NORTH
𝑦-> EAST
𝑧 -> ALTITUDE (+)

Each (𝑥,𝑦,𝑧) coordinate is the center of the obstacle. 𝛿𝑥, 𝛿𝑦, 𝛿𝑧 are the half widths of the obstacles, meaning for example that an obstacle with (𝑥=37,𝑦=12,𝑧=8) and (𝛿𝑥=5,𝛿𝑦=5,𝛿𝑧=8) is a 10 x 10 m obstacle that is 16 m high and centered at the point (37,12,8)
Given a map like this, the free space in the (𝑥,𝑦) plane is a function of altitude, and you can plan a path around an obstacle, or simply fly over it! You'll extend each obstacle by a safety margin to create the equivalent of a 3 dimensional configuration space. Your task is to extract a 2D map of your configuration space for a particular altitude, where each value is assigned either a 0 or 1 representing feasible or infeasible (obstacle) spaces respectively.



***************VIP************************
[-310.2389  -439.2315  85.5     5.0          5.0           85.5]               --> Obstacle information
[Xc            Yc      Zc      (+/-)WidX    (+/-)WidY     (+/-heightZ]
Xcentre = -310     Xright= -310+5 =-305  Xleft = -310-5 = -315    So Obstacle boundary along X = (-315, -305) 
SimillRY for Y also..                                                                    
For Z,  Zcentre = 85.5, Zdown= 85.5-85.5 = 0m ground   Zup = 85.5+85.5= 171m height.      Using all these XYZ values represent the grid boundary of obstracle

Likewise calculate for each obstacle.
'''

import numpy as np 
import matplotlib.pyplot as plt

# %matplotlib inline

plt.rcParams["figure.figsize"] = [12, 12]
filename = '/home/darshan/udacity/Motion planning/2.Flying car rep/colliders.csv'
# Read in the data skipping the first two lines.  
# Note: the first line contains the latitude and longitude of map center
# Where is this??
data = np.loadtxt(filename, delimiter=',',dtype='Float64',skiprows=2)
print(data)

# Static drone altitude (metres)
drone_altitude = 10

# Minimum distance required to stay away from an obstacle (metres)
# Think of this as padding around the obstacle.
safe_distance = 3

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))             #This is complete empty grid dimmentions(Grid means size of map)   ie,921*921

    # Populate the grid with obstacles                **************************** Refer the Doc section above(VIP) [loading the obstacles into the grid map]
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    print(int(north_min), int(east_min))
    return grid

grid = create_grid(data, drone_altitude, safe_distance)

# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower') 

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

#Play around with the drone_altitude and safe_distance values to get a feel for how it changes the map.
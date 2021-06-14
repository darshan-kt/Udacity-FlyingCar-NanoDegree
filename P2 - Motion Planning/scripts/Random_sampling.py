# -*- coding: utf-8 -*-

'''
Random Sampling
In this notebook you'll work with the obstacle's polygon representation itself.

Your tasks will be:

    Create polygons.
    Sample random 3D points.
    Remove points contained by an obstacle polygon.

Checking Condition:
Recall, a point (𝑥,𝑦,𝑧) collides with a created obstacle polygon, if the (𝑥,𝑦) coordinates are contained(present) by the polygon and the 𝑧 coordinate (height) is less than the height of the polygon.
'''

import time
import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Polygon, Point
from test_grid import create_grid

plt.rcParams['figure.figsize'] = 12, 12
# This is the same obstacle data from the previous lesson.
filename = '/home/darshan/udacity/Motion planning/Moving into 3D/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(len(data))

# Real work start from here ------------------
# 1. CREATE POLYGON

def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        # Extract the 4 corners of each obstacle
        #obstacle_boundary= [ Yc+widY, Yc-widY, Xc+widX, Xc+widX ]    ---> Rep formula
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        # corners = [(y0,x0), (y0,x1), (y1,x1), (y1, x0)       --> Polygon corners
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        # Height  (Zup = Zc+Zh  Zdown is not considered here because Zdown is always 0 and we cann't navigate under Zc)
        height = alt + d_alt
        
        p = Polygon(corners)
        polygons.append((p, height))
        
    return polygons

polygons = extract_polygons(data)
print(len(polygons))


# 2. SAMPLING 3D POINTS
# Now that we have the extracted the polygons, we need to sample random 3D points. Currently we don't know suitable ranges for x, y, and z. Let's figure out the max and min values for each dimension.

xmin = np.min(data[:, 0] - data[:, 3])        # Minimum x value = min value in all elements of 1st coloum - min value along 4th column
xmax = np.max(data[:, 0] + data[:, 3])        # Miximum x value

ymin = np.min(data[:, 1] - data[:, 4])        # Minimum y value
ymax = np.max(data[:, 1] + data[:, 4])        # Miximum y value

zmin = 0
zmax = 10    #Height is fixed 10m

print("X")
print("min = {0}, max = {1}\n".format(xmin, xmax))

print("Y")
print("min = {0}, max = {1}\n".format(ymin, ymax))

print("Z")
print("min = {0}, max = {1}".format(zmin, zmax))



# Next uniformly distributes the samples between Xmin to Xmax, Ymin to Ymax and Zmin to Zmax
num_samples = 100
xvals = np.random.uniform(xmin, xmax, num_samples)
yvals = np.random.uniform(ymin, ymax, num_samples)
zvals = np.random.uniform(zmin, zmax, num_samples)

samples = np.array(list(zip(xvals, yvals, zvals)))   #listing randomly sampled values along xyz axises
# print(samples)



# 3. REMOVING THE POITNS COLLIDING WITH OBSTRACLESS
# Prior to remove a point we must determine whether it collides with any obstacle. 
# Complete the collides function below. It should return True if the point collides with any obstacle and False if no collision is detected.

def collides(polygons, point):        #point is xyz value, polygen is obstacle boundary
    Random_sample = point
    for (p, height) in polygons:
        #if obstracle point inside polygon and obstracle height greater return True (We don't need these points)
        if p.contains(Point(Random_sample[0], Random_sample[1])) and height >= Random_sample[2]:   #refer previous polygen_test.py script
            return True

        return False
        
# Use collides for all points in the sample.
t0 = time.time()
to_keep = []
for point in samples:
    if not collides(polygons, point):  #A/c to condtion we need False condtioned points(ie, Outside the obstacle polygon and less than height of obstacle )
        to_keep.append(point)

time_taken = time.time() - t0
print("Time taken {0} seconds".format(time_taken))
# print(len(to_keep))



#4. Visualize and Plotting purpose
grid = create_grid(data, zmax, 1)

fig = plt.figure()

plt.imshow(grid, cmap='Greys', origin='lower')

nmin = np.min(data[:, 0])      
emin = np.min(data[:, 1])

# draw points
all_pts = np.array(to_keep)         #These are valid points and needs to display these
north_vals = all_pts[:,0]
east_vals = all_pts[:,1]
# print(east_vals , emin, east_vals - emin)
plt.scatter(east_vals - emin, north_vals - nmin, c='red')

plt.ylabel('NORTH')
plt.xlabel('EAST')

plt.show()
# -*- coding: utf-8 -*-

'''
Youtube video: https://www.youtube.com/watch?v=fy40y3UFkNE

The KD Tree is a space-partitioning data structure, which allows for fast search queries.
The KD Tree achieves this by cutting the search space in half on each step of a query.
If you're familiar with "big O notation", this brings the "total search time down" to O(m∗log(n)) from O(m∗n), 
where m is the number of elements to compare to and n is the number of elements in the KD Tree. 
So for example, if you want to find the closest neighbor to a single point, m=1 and n is equal to the total number of potential neighbors.

The Python Scikit-Learn (sklearn) library has an easy to use implementation of KD Trees that we'll be introducing in this exercise.
'''

#Import the KDTree and numpy
from sklearn.neighbors import KDTree
import numpy as np 

#Generate some random 3-D points
# np.random.seed(0)
# points = np.random.random((10, 3))
# print(points)

# #Cast the points into a KDTree data structure
# tree = KDTree(points)

# #Extract the indices of 3 closet points
# #Note: need to cast search point as a list and return 0th element only to get back list of indices
# idxs = tree.query([points[0]], k=3, return_distance=False)[0]

# #indices of 3 closet neighbors
# print(idxs)

rng = np.random.RandomState(0)
X = rng.random_sample((15, 3))  # 10 points in 3 dimensions
print(X)
tree = KDTree(X, leaf_size=2)              
dist, ind = tree.query(X[:1], k=3)                
print(ind)  # indices of 3 closest neighbors

print(dist)  
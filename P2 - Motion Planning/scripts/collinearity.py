# -*- coding: utf-8 -*-

"""
FInding out the collinear(stright lined) points for 2D

Area=x1​(y2​−y3​)+x2​(y3​−y1​)+x3​(y1​−y2​)
"""

import numpy as np
#define 2D points
# p1 = np.array([1,2])
# p2 = np.array([2,4])
# p3 = np.array([3,4])

'''
# Formula method
def area(p1,p2,p3):
    A = p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]- p1[1]) + p3[0]*(p1[1]- p2[1])
    if A == 0:
        print(p1,p2,p3, "are linnear")
    else:
        print(p1,p2,p3, "are non-linnear") 
area(p1,p2,p3)
'''



# This is direct approch using matrix concept
# Define some 2D points
p1 = np.array([1.001, 2.002])
p2 = np.array([1.999, 3.001])
p3 = np.array([2.99, 4.002])

# Define a simple function to add a z coordinate of 1
def point(p):
    return np.array([p[0], p[1], 1.])

# Add points as rows in a matrix
mat = np.vstack((point(p1), point(p2), point(p3)))
# Compute and print the determinant of the matrix(or Area of matrix)
det = np.linalg.det(mat)
print(det)


# Include a new term called epsilon to set thresold
collinear = False
epsilon = 1e-2  #0.01  

# Compare the absolute value of the determinant with epsilon
if np.abs(det) < epsilon:      #0.008 < 0.01
    collinear = True
    print("points are collinear")
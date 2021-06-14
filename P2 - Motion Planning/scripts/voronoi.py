#Import numpy and Voronoi method and plotting routines
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt


#Generate 50 random points with integer values between 0 and 49
points = np.random.randint(50, size=(50,2))

# Extract the Voronoi diagram
graph = Voronoi(points)   #This function creates tthe voronoi diagram based on points/seeds
# plot it up!
voronoi_plot_2d(graph)
plt.show()        # now can see the graph for 50 random points



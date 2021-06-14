import sys
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
from queue import PriorityQueue


plt.rcParams['figure.figsize'] = 12, 12
# Read in the obstacle data
filename = '/home/darshan/udacity/Motion planning/3.Grid to Graph/Graph_search_city/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# Starting and goal positions in *(north, east)*
start_ne = (25,  100)
goal_ne = (750., 370.)

# Static drone altitude (metres)
drone_altitude = 5
safety_distance = 3

# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
print(len(edges))   #Now got the Graph with edges(lines) and nodes

#------------Till now graph generation was completed--------------

plt.imshow(grid, origin='lower', cmap='Greys') 


# Plot the edges on top of the grid along with start and goal locations. (if required) 
# for e in edges:
#     p1 = e[0]
#     p2 = e[1]
#     plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
# plt.plot(start_ne[1], start_ne[0], 'rx')
# plt.plot(goal_ne[1], goal_ne[0], 'rx')

# plt.xlabel('EAST')
# plt.ylabel('NORTH')
# plt.show()


'''
We now have a graph, well at least visually. The next step is to use the networkx to create the graph. NetworkX is a popular library handling anything and everything related to graph data structures and algorithms.

NOTE: In the initial import above it was imported with the nx alias.

You're encouraged to read the documentation but here's a super quick tour:

    Create a graph:

G = nx.Graph()

    Add an edge:

p1 = (10, 2.2)
p2 = (50, 40)
G = nx.add_edge(p1, p2)

3 Add an edge with a weight:

p1 = (10, 2.2)
p2 = (50, 40)
dist = LA.norm(np.array(p2) - np.array(p1))
G = nx.add_edge(p1, p2, weight=dist)
'''

# set to the Euclidean distance between the points
G = nx.Graph()
for e in edges:
    p1 = e[0]
    p2 = e[1]
    dist = LA.norm(np.array(p2) - np.array(p1))
    G.add_edge(p1, p2, weight=dist)
    
# For the cost function, you can now think of the cost of moving between two nodes as a function of the distance between those nodes in the graph.
def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))

def a_star(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

'''
Solution

This solution consists of two parts:

    Find the closest point in the graph to our current location, same thing for the goal location.
    Compute the path from the two points in the graph using the A* algorithm.
    Feel free to use any of the path pruning techniques to make the path even smaller!

'''

def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph` to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

start_ne_g = closest_point(G, start_ne)
goal_ne_g = closest_point(G, goal_ne)
# print(start_ne_g)
# print(goal_ne_g)

path, cost = a_star(G, heuristic, start_ne_g, goal_ne_g)
# print(len(path))

plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
    
plt.plot([start_ne[1], start_ne_g[1]], [start_ne[0], start_ne_g[0]], 'r-')
for i in range(len(path)-1):
    p1 = path[i]
    p2 = path[i+1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
plt.plot([goal_ne[1], goal_ne_g[1]], [goal_ne[0], goal_ne_g[0]], 'r-')
    
plt.plot(start_ne[1], start_ne[0], 'gx')
plt.plot(goal_ne[1], goal_ne[0], 'gx')

plt.xlabel('EAST', fontsize=20)
plt.ylabel('NORTH', fontsize=20)
plt.show()
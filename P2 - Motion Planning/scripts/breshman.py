import numpy as np 
import matplotlib.pyplot as plt

#Using formula approach

plt.rcParams['figure.figsize'] = 12, 120

def bres(p1, p2):
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []
    
    # Here's a quick explanation in math terms of our approach
    # First, set dx = x2 - x1 and dy = y2 - y1
    dx, dy = x2 - x1, y2 - y1
    # Then define a new quantity: d = x dy - y dx.
    # and set d = 0 initially
    d = 0
    # The condition we care about is whether 
    # (x + 1) * m < y + 1 or moving things around a bit: 
    # (x + 1) dy / dx < y + 1 
    # which implies: x dy - y dx < dx - dy
    # or in other words: d < dx - dy is our new condition
    
    # Initialize i, j indices
    i = x1
    j = y1
    
    while i < x2 and j < y2:
        cells.append([i, j])
        if d < dx - dy:
            d += dy
            i += 1
        elif d == dx - dy:
            # uncomment these two lines for conservative approach
            #cells.append([i+1, j])
            #cells.append([i, j+1])
            d += dy
            i += 1  
            d -= dx
            j += 1
        else:
            d -= dx
            j += 1

    return np.array(cells)

p1 = (0, 0)
p2 = (5, 5)

cells = bres(p1, p2)
# print(cells)

plt.plot([p1[0], p2[0]], [p1[1], p2[1]])


for q in cells:
    plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
    plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Integer based Bresenham algorithm")
plt.show()
    
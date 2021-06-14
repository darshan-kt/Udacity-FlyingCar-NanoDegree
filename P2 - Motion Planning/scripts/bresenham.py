import pybresenham
import matplotlib.pyplot as plt

# Install pybresenham library
# pip install pybresenham

# Note: you can run this for any (x1, y1, x2, y2)
line = (0, 0, 7, 5)

cells = list(pybresenham.line(line[0], line[1], line[2], line[3]))
print(cells)      #This diplays the all intermidiate points


#This for visualization purpose
plt.plot([line[0], line[2]], [line[1], line[3]])

# This is extra to just design grid cells for passing line of points. U can comment this if u not need
for q in cells:
    plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
    plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Python package Bresenham algorithm")
plt.show()
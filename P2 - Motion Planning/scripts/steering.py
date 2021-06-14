
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = 12, 12

# limit the steering angle range
MAX_STEERING_ANGLE = np.deg2rad(30)
# Set the width of the Gaussian we'll draw angles from
ANGLE_STDDEV = np.deg2rad(3)

# NOTE TO THE READER:
# This steer() function is not meant to be a particularly clever
# method of steering from x1 to x2.  Rather this is just a way 
# of guessing which way we might steer in order to make rough
# progress toward x2. We'll get to better steering later!

def steer(x1, x2):
    theta = x1[2]
    
    # calculate angle difference between the direction toward x2
    # and our current orientation
    angle = np.arctan2(x2[1] - x1[1], x2[0] - x1[0]) - theta
    
    # Now we'll just do something rather naive and assume that if we know the 
    # direction of x2, we'll set the steering angle roughly in that direction
    # sample angle from a gaussian centered around the direction toward x2
    angle = np.random.normal(angle, ANGLE_STDDEV)
    
    # clip angle value between min and max
    angle = np.clip(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
                
    return angle

def simulate(state, angle, v, dt):
    x = state[0]
    y = state[1]
    theta = state[2]    
    
    nx = x + v*np.cos(theta)*dt
    ny = y + v*np.sin(theta)*dt
    ntheta = theta + v*np.tan(angle)*dt
    
    return [nx, ny, ntheta]

dt = 0.2
T = 10

start = [0, 0, 0]
goal = [10, 0, 0]

angles = [steer(start, goal) for _ in range(100)]
lines = []

for angle in angles:
    line = [start]
    state = np.copy(start)
    v = np.random.uniform(0, 1)
    for _ in np.arange(0, T, dt):
        state = simulate(state, angle, v, dt)
        line.append(state)
    lines.append(line)
    
lines = np.array(lines)
print(lines.shape)

for i in range(lines.shape[0]):
    plt.plot(lines[i, :, 0], lines[i, :, 1], 'b-')
plt.plot(start[0], start[1], 'bo')
plt.plot(goal[0], goal[1], 'ro')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()
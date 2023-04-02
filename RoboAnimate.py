import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Load odometry data from file
data = np.loadtxt('myodom2.txt', delimiter=',')

# Calculate heading from differences in x and y coordinates using i-3rd element
dx = np.diff(data[:, 0])
dy = np.diff(data[:, 1])
headings = np.arctan2(dy, dx)

# Prepend initial heading as 0
headings = np.insert(headings, 0, 0)

# Set up figure and axes
fig, ax = plt.subplots(figsize=(3, 3))
ax.set_xlim(-3.5, 3.5)
ax.set_ylim(-3.5, 3.5)

# Create trail objects
trail, = ax.plot([], [], lw=2, c='b')
head, = ax.plot([], [], 'ro', markersize=4)

obstacles = [(-1.25, 1), (0, 1), (1.25, 1),
             (-1.25, 0), (0, 0), (1.25, 0),
             (-1.25, -1), (0, -1), (1.25, -1)]
for obs in obstacles:
    circle = plt.Circle(obs, 0.1, color='green')
    ax.add_artist(circle)


# Initialize function for animation
def animate(i):
    x, y = data[i, :2]
    heading = headings[i]
    trail.set_data(data[:i, 0], data[:i, 1])
    head.set_data(x, y)
    head.set_markeredgecolor('r')
    head.set_markerfacecolor('r')
    head.set_markerfacecoloralt('w')
    head.set_markersize(4)
    head.set_markeredgewidth(1)
    head.set_alpha(0.8)
    head.set_linestyle('None')
    head.set_drawstyle('steps')
    head.set_solid_capstyle('round')
    return trail, head


# Call animation function
ani = FuncAnimation(fig, animate, frames=len(data), interval=4, blit=True)

plt.show()

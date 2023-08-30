import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sympy import symbols, Eq, solve, exp, I, re, im
import math

class Linkage:
    def __init__(self, lengths, connections):
        self.lengths = lengths
        self.connections = connections
        # self.fixed_links = fixed_links
        self.num_links = len(lengths)
        self.angles = np.zeros(self.num_links)
    
    def forward_kinematics(self):
        points = np.zeros(self.num_links+1, dtype=complex)
        for i in range(1, self.num_links+1):
            parent_link = self.connections[i-1]
            points[i] = points[parent_link] + self.lengths[i-1] * np.exp(1j * np.sum(self.angles[:i]))
        
        x = np.real(points)
        y = np.imag(points)
        return x, y
    
    def set_angles(self, angles):
        self.angles = angles

class Animation:
    def __init__(self, linkage):
        self.linkage = linkage
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_aspect('equal')
        self.line, = self.ax.plot([], [], 'o-', lw=2)
    
    def update(self, frame):
        theta = frame * 0.05
        angles = np.array([theta, -2*theta, theta, -theta])
        self.linkage.set_angles(angles)
        x, y = self.linkage.forward_kinematics()
        self.line.set_data(x, y)
        return self.line,
    
    def animate(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(200), blit=True)
        plt.show()

# Create the linkage object
lengths = [1.0, 2.0, 1.5, 1.0]
connections = [0, 1, 2, 3]
fixed_links = [0, 3]  # Fix the first and last links' one end on the ground
linkage = Linkage(lengths, connections, fixed_links)

# Create the animation object
animation = Animation(linkage)

# Run the animation
animation.animate()
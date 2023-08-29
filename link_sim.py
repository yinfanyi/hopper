import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Linkage:
    def __init__(self, lengths):
        self.lengths = lengths
        self.num_links = len(lengths)
        self.angles = np.zeros(self.num_links)
    
    def forward_kinematics(self):
        x = np.zeros(self.num_links+1)
        y = np.zeros(self.num_links+1)
        for i in range(1, self.num_links+1):
            x[i] = x[i-1] + self.lengths[i-1] * np.cos(np.sum(self.angles[:i]))
            y[i] = y[i-1] + self.lengths[i-1] * np.sin(np.sum(self.angles[:i]))
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

# 创建连杆对象
lengths = [1.0, 2.0, 1.5, 1.0]
linkage = Linkage(lengths)

# 创建动画对象
animation = Animation(linkage)

# 运行动画
animation.animate()
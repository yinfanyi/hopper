import mujoco as mj
import mediapy as media
import matplotlib.pyplot as plt
import time
from datetime import datetime
import itertools
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import threading


from mujoco_base import MuJoCoBase
from hopper import *

def simulate_hopper(sim: Hopper1, tendon_stiffness_data):
    sim.model.tendon_stiffness[0] += 100
    tendon_stiffness_data.append(sim.model.tendon_stiffness[0])
    sim.reset()
    sim.simulate()

def main():
    xml_path = "./xml/hopper1/scene2.xml"
    i = 0
    tendon_stiffness_data = []
    max_height_data = []
    threads = []
    
    sim = Hopper1(xml_path)
    sim.simend = 2
    sim.is_plot_data = False
    sim.is_render = False
    
    while i < 3:
        t = threading.Thread(target=simulate_hopper, args=(sim, tendon_stiffness_data))
        t.start()
        threads.append(t)
        i += 1

    for t in threads:
        t.join()
    print(tendon_stiffness_data)
    # print(max_height_data)
    
    # sim.initial_state()

if __name__ == "__main__":
    main()

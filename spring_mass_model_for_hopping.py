import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import fsolve
# from sympy import symbols, Eq, solve, exp, I, re, im
import math
from typing import Optional
import time

# 大目标：仿真任意跳跃机器人的跳跃情况
# 1.实现串联连杆机构的正逆运动学求解
# 2.实现串联连杆机构的运动动画
# 3.实现添加弹簧和能量计算
# 4.弹簧动画
# 5.任意机构的正逆运动学求解
# 5.动力学仿真
# 6.跳跃模拟


class Spring:
    def __init__(self, mass: float, initial_length: float, stiffness: float) -> None:
        self.mass = mass
        self.stiffness = stiffness
        self.initial_length = initial_length
    
    @property
    def energy(self, current_length: float):
        return 0.5 * self.stiffness * abs(current_length - self.initial_length)**2
    
    # 弹簧对连杆的影响：求解出连杆处于势能最小的位置 
    # 双稳态？

class Linkage:
    def __init__(self, lengths:list, driving_link_index:list, initional_angle:Optional[np.array]=None):
        """只考虑串联，最后一个杆为地面杆
        """
        self.lengths = lengths
        # self.connections = connections
        self.num_links = len(lengths)
        self.connections = [i for i in range(self.num_links)]
        self.angles = np.zeros(self.num_links)
        self.initional_angle = initional_angle
        self.driving_link_index = driving_link_index
        self.passive_link_index = [num for num in self.connections if num not in self.driving_link_index]
    
    def forward_kinematics(self):
        """获得当前所有构件的位置
        """
        points = np.zeros(self.num_links+1, dtype=complex)
        for i in range(1, self.num_links+1):
            parent_link = self.connections[i-1]
            points[i] = points[parent_link] + self.lengths[i-1] * np.exp(1j * np.sum(self.angles[:i]))
        x = np.real(points)
        y = np.imag(points)
        return x, y
    
    def inverse_kinematics(self, driving_target_angles:list, current_angle:Optional[list]=None):
        '''逆运动学：输入主动件的输入转角，得到所有构件的转角
        
            参数：
            driving_target_angles：主动件的转角，弧度制。
            current_angle:当前所有关节的角度，弧度制。
            
            输出：
            目标所有关节的角度，弧度制
            
            存在问题：
            1. 只能求串联机构
            2. 没有错误判断 
        '''
        if current_angle == None:
            current_angle = self.initional_pose
        assert len(driving_target_angles)==len(self.driving_link_index), "主动件转动角度的个数必须与主动件个数相同"
        
        X0 = []
        for index in self.passive_link_index:
            X0.append(current_angle[index])
        
        def f(X=[0., 0.]):
            # 关节之间约束关系，分解成x和y方向，因此只有两个未知数
            i = 0
            result_x = 0.
            result_y = 0.
            for angle in driving_target_angles:
                j = self.driving_link_index[i]
                result_x += self.lengths[j] * math.cos(angle)
                result_y += self.lengths[j] * math.sin(angle)
                i = i + 1
            
            result_x += self.lengths[self.passive_link_index[0]] * math.cos(X[0])\
                + self.lengths[self.passive_link_index[1]] * math.cos(X[1])
            result_y += self.lengths[self.passive_link_index[0]] * math.sin(X[0])\
                + self.lengths[self.passive_link_index[1]] * math.sin(X[1])
            return [result_x, result_y]
        
        start = time.time()
        # 使用数值解，求解和初始位姿接近的解，但求解结果没有保证
        results = fsolve(f, X0)
        print('逆解结果为：', results)
        end = time.time()
        print('time',end - start)

        target_angles = [0] * self.num_links
        i = 0
        for index in self.driving_link_index:
            target_angles[index] = driving_target_angles[i]
            i += 1
            
        i = 0
        for index in self.passive_link_index:
            target_angles[index] = results[i]
            i += 1

        return target_angles
    
    def set_angles(self, angles):
        self.angles = angles

    def check_valid(self):
        """检验当前关节角是否能实现，能实现返回0，不能实现返回1
        """
        pass

if __name__ == '__main__':
    lengths = [1.0, 1.0, 1., 1.0]
    driving_link_index = [0,3]
    linkage = Linkage(lengths, driving_link_index)
    print(linkage.inverse_kinematics(driving_target_angles=[3.14/2, -math.pi], 
                                     current_angle=[3.14/2, 0, 0, -math.pi]))
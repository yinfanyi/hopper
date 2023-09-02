import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import fsolve
# from sympy import symbols, Eq, solve, exp, I, re, im
import math
# import cmath
from typing import Optional
import time

# 大目标：仿真任意跳跃机器人的跳跃情况

# 如何让点既能在外面定义，也能在

# 3.实现添加弹簧和能量计算
## 弹簧连接点问题：如果弹簧不是连在关节点上，如何定义
## 如果弹簧是和弹簧相连，怎么定义，三根弹簧并联？

# 4.弹簧动画


# 5.任意机构的正逆运动学求解

# 6.动力学仿真

# 7.跳跃模拟


class Point:
    def __init__(self, name: str, position:np.ndarray) -> bool:
        '''定义一个点类，让连杆构件的的两端和弹簧构件继承这些点，方便保持不同构件之间连接拓扑机构在机构运动后保持不变
            参数：
            name 名字，指定点的名字
            pos 点在世界坐标的位置，类型：3*1 np.array矩阵
        '''
        self.name = name
        if self.check_position_array_length(position):            
            self.position = position
        else:
            print('点定义的世界坐标不合法')
            return False

    def check_position_array_length(self, position:np.ndarray) -> bool:
        if position.shape != (3, 1):
            return False
        return True

class Line:
    def __init__(self, name: str, start_point: Point, 
                 end_point: Point, fix_points: list[Point]) -> bool:
        '''定义一个线类，用于模拟杆件，方便点类附着在上面
            参数：
            name 名字，指定点的名字
        '''
        self.name = name
        self.update_points(start_point, end_point)
        self.fix_points = fix_points
    
    def update_points(self, start_point: Point, end_point: Point):
        self.start_point = start_point
        self.end_point = end_point
        self.current_length = np.linalg.norm(end_point.position - 
                                     start_point.position)
        


class Spring:
    def __init__(self, mass: float, initial_length: float, 
                 stiffness: float, start_point: Point, end_point: Point) -> None:
        self.mass = mass
        self.stiffness = stiffness
        self.initial_length = initial_length
        self.update_points(start_point, end_point)
        
    def update_points(self, start_point: Point, end_point: Point):
        self.start_point = start_point
        self.end_point = end_point
        self.current_length = np.linalg.norm(end_point.position - 
                                     start_point.position)

    @property
    def energy(self):
        # 需要显式更新弹簧的位置后再获取能量，否则只能获取前一个时刻的能量值
        return 0.5 * self.stiffness * abs(self.current_length - self.initial_length)**2
    
    # 弹簧对连杆的影响：求解出连杆处于势能最小的位置 
    # 双稳态？

class Linkage:
    def __init__(self, lengths:list, driving_link_index:list, initional_angle:Optional[np.array]=None):
        """只考虑串联的单回路连杆，可以设定任意连杆为主动件
        """
        self.lengths = lengths
        # self.connections = connections
        self.num_links = len(lengths)
        self.connections = [i for i in range(self.num_links)]
        self.initional_angle = initional_angle
        self.angles = initional_angle
        self.driving_link_index = driving_link_index
        self.passive_link_index = [num for num in self.connections if num not in self.driving_link_index]
    
    def forward_kinematics(self):
        """获得当前所有构件的位置
        """
        points = np.zeros(self.num_links+1, dtype=complex)
        for i in range(1, self.num_links+1):
            # parent_link = self.connections[i-1]
            points[i] = points[i-1] + self.lengths[i-1] * np.exp(1j * self.angles[i-1])
        x = np.real(points)
        y = np.imag(points)
        return x, y
    
    def inverse_kinematics(self, driving_target_angles:list, current_angles:Optional[list]=None) -> Optional[list]:
        '''逆运动学：输入主动件的输入转角，得到所有构件的转角
        
            参数：
            driving_target_angles：主动件的转角，弧度制。
            current_angles:当前所有关节的角度，弧度制。
            
            输出：
            目标所有关节的角度，弧度制
            
            存在问题：
            1. 只能求串联机构
        '''
        if current_angles is None:
            current_angles = self.initional_angle

        assert len(driving_target_angles)==len(self.driving_link_index), "主动件转动角度的个数必须与主动件个数相同"
        
        X0 = []
        for index in self.passive_link_index:
            X0.append(current_angles[index])
        
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
        
        # 使用数值解，求解和初始位姿接近的解，但求解结果没有保证
        results = fsolve(f, X0)
        target_angles = [0] * self.num_links
        i = 0
        for index in self.driving_link_index:
            target_angles[index] = driving_target_angles[i]
            i += 1
            
        i = 0
        for index in self.passive_link_index:
            target_angles[index] = results[i]
            i += 1

        target_angle_limited = []
        for target_angle in target_angles:
            # 将角度限制在-pi到pi范围内
            while abs(target_angle) > math.pi:
                if target_angle > 0:
                    target_angle += -2 * math.pi
                elif target_angle < 0:
                    target_angle += 2 * math.pi
            target_angle_limited.append(target_angle)
                    
        return target_angle_limited
    
    def set_angles(self, angles:list)->bool:
        """设置当前所有关节角的大小，单位：弧度制
        """
        assert len(angles) == self.num_links, "请输入连杆对应数目的关节角！"
        
        if self.check_valid(angles) == False:
            print('目标位置不可达，无法设置该关节角')
            return False
        
        self.angles = angles
        return True

    def check_valid(self, angles:list):
        """检验该关节角是否能实现，能实现返回0，不能实现返回1
        """
        result = 0j # 用复数求回路是否闭合，即向量相加等于零
        i = 0
        for angle in angles:
            result += self.lengths[i] * np.exp(1j * angle)
            i += 1
        if abs(result) > 0.1:
            return False
        return True
        
class Animation:
    def __init__(self, linkage: Linkage):
        self.linkage = linkage
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-4, 4)
        self.ax.set_aspect('equal')
        self.line, = self.ax.plot([], [], 'o-', lw=2)
    
    def update(self, frame):
        theta = frame * 0.05
        target_angles = self.linkage.inverse_kinematics(driving_target_angles=[theta, -math.pi], 
                                                 current_angles=self.linkage.angles)
        self.linkage.set_angles(target_angles)
        x, y = self.linkage.forward_kinematics()
        print(f'x:{x}\ny:{y}')
        print(f'target_angles:\n{target_angles}')
        print('----------------------------------------------')
        self.line.set_data(x, y)
        return self.line,
    
    def animate(self):
        ani = FuncAnimation(self.fig, self.update, frames=range(200), blit=True)
        plt.show()
        
if __name__ == '__main__':
    lengths = [1.0, 1.0, 1., 1.0]
    driving_link_index = [0,3]
    initional_angle = [3.14/2, 0, -math.pi/2, -math.pi]
    
    linkage = Linkage(lengths, driving_link_index,initional_angle)
    
    animation = Animation(linkage)
    animation.animate()
    
    # angles = linkage.inverse_kinematics(driving_target_angles=[3.14/2, -math.pi], 
    #                                  current_angles=[3.14/2, 0, -math.pi/2, -math.pi])
    # print(angles)
    # print(linkage.set_angles(angles))
    # pprint(linkage.forward_kinematics())
    

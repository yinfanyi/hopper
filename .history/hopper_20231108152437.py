import mujoco as mj
import mediapy as media
import matplotlib.pyplot as plt
import time
import itertools
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv


from mujoco_base import MuJoCoBase

# TODO:
# 2.建模完成运动链闭环结构；弹簧大小、位置、性能设置
# 3.保存运动时传感器参数并绘图
# 4.在窗口或代码中修改模型

# 1、实时画图 结束时显示所有的信息
# 2、代码内修改模型
# 3、获取site s_left的位置信息
# 4、显示世界坐标轴等各种信息

# FSM_AIR1 = 0
# FSM_STANCE1 = 1
# FSM_STANCE2 = 2
# FSM_AIR2 = 3

times = []
kinetic_energies = []
potential_energies = []
total_energies = []
spring_length = []

class Hopper1(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0
        # self.fsm = None
        # self.step_no = 0

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])
        self.data.qpos = [0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0]
        
        # self.model.opt.gravity = [0,0,-1]

        # self.fsm = FSM_AIR1 # 有限状态机（Finite State Machine）
        # self.step_no = 0

        # pservo-hip
        # self.set_position_servo(0, 100)

        # vservo-hip
        # self.set_velocity_servo(1, 10)

        # pservo-knee
        # self.set_position_servo(2, 1000)

        # vservo-knee
        # self.set_velocity_servo(3, 0)

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """
        # return
        # 之后不要直接读取位置，而是通过传感器读取位置，给传感器扰动，以符合现实传感器波动导致机器不稳的情况
        # z_foot = data.sensordata[0]
        body_no = 3
        z_foot = data.xpos[body_no, 2]
        vz_torso = data.qvel[1]
        
        mj.mj_energyPos(model, data)
        mj.mj_energyVel(model, data)
        
        # Lands on the ground
        # if self.fsm == FSM_AIR1 and z_foot < 0.05:
        #     self.fsm = FSM_STANCE1
        
        # # Moving upward
        # if self.fsm == FSM_STANCE1 and vz_torso > 0.0:
        #     self.fsm = FSM_STANCE2

        # # Take off
        # if self.fsm == FSM_STANCE2 and z_foot > 0.05:
        #     self.fsm = FSM_AIR2
        
        # # Moving downward
        # if self.fsm == FSM_AIR2 and vz_torso < 0.0:
        #     self.fsm = FSM_AIR1
        #     self.step_no += 1
        
        # if self.fsm == FSM_AIR1:
        #     self.set_position_servo(2, 100)
        #     self.set_velocity_servo(3, 10)

        # if self.fsm == FSM_STANCE1:
        #     self.set_position_servo(2, 1000)
        #     self.set_velocity_servo(3, 0)

        # if self.fsm == FSM_STANCE2:
        #     self.set_position_servo(2, 1000)
        #     self.set_velocity_servo(3, 0)
        #     data.ctrl[0] = -0.2

        # if self.fsm == FSM_AIR2:
        #     self.set_position_servo(2, 100)
        #     self.set_velocity_servo(3, 10)
        #     data.ctrl[0] = 0.0

    def keyboard(self, window, key, scancode, act, mods):
            if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
                mj.mj_resetData(self.model, self.data)
                mj.mj_forward(self.model, self.data)
            if act == glfw.PRESS and key == glfw.KEY_A:
                # 在这里增加逻辑
                # 改变模型、太
                pass

    def simulate(self):
        start_time = time.time()
        while not glfw.window_should_close(self.window):
            simstart = self.data.time
            
            current_time = time.time()
            
            while (self.data.time - simstart < 1.0/60.0):
                # Step simulation environment
                mj.mj_step(self.model, self.data)
                
            times.append(self.data.time)
            kinetic_energies.append(self.data.energy[0])
            potential_energies.append(self.data.energy[1])
            total_energies.append(self.data.energy[0] + self.data.energy[1])
            spring_length.append(abs(self.data.site_xpos[1,0]-self.data.site_xpos[1,0]))
            plt.figure(1)
            plt.clf()
            plt.plot(times, kinetic_energies, label='Kinetic Energy')
            plt.plot(times, potential_energies, label='Potential Energy')
            plt.plot(times, total_energies, label='Total Energy')
            plt.plot(times, spring_length, label='spring_length')
            plt.xlabel('Time')
            plt.ylabel('Energy')
            plt.legend()
            # plt.show(block=False)
            plt.draw()
            plt.pause(0.0001)

            if self.data.time >= self.simend:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            self.cam.lookat[0] = self.data.qpos[0]
            
            # scene_option = mj.MjvOption()
            # scene_option.flags[mj.mjtVisFlag.mjVIS_JOINT] = True
            # scene_option.frame = mj.mjtFrame.mjFRAME_GEOM
            # scene_option.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = True
            
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()
            
            # print(f"cam lookat{self.cam.lookat}")
            if (current_time - start_time) % 2 < 0.1: 
                # print(f"data qpos{self.data.qpos}")
                # print(f"self.model.actuator_gainprm {self.model.actuator_gainprm}")
                print(f"time{self.data.time},\n动能{self.data.energy[0]},势能{self.data.energy[1]}\n,总能量{self.data.energy[0]+self.data.energy[1]}")
                # print(f"传感器数据{self.data.sensordata}")
                print(f"节点的位置{self.data.site_xpos}")
                
        glfw.terminate()
    
    def initial_state(self):
        # self.model.body_pos[1,2] = 3.5
        # self.cam.lookat[0] = self.data.qpos[0]
        mj.mj_step(self.model, self.data)
        # 可以通过 self.model.body_pos 来改变物体的初始位置
        print(f"body_pos\n{self.model.body_pos}")
        while not glfw.window_should_close(self.window):
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                                mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)
            glfw.swap_buffers(self.window)
            glfw.poll_events()
        glfw.terminate()
        
    # def set_position_servo(self, actuator_no, kp):
    #     self.model.actuator_gainprm[actuator_no, 0] = kp
    #     self.model.actuator_biasprm[actuator_no, 1] = -kp
    
    # def set_velocity_servo(self, actuator_no, kv):
    #     self.model.actuator_gainprm[actuator_no, 0] = kv
    #     self.model.actuator_biasprm[actuator_no, 2] = -kv

def main():
    xml_path = "./xml/hopper1/scene.xml"
    sim = Hopper1(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()

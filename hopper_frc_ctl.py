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
from tqdm import tqdm

from mujoco_base import MuJoCoBase

FSM_AIR = 0
FSM_STANCE = 1
FSM_TAKEOFF = 2

class HopperData:
    def __init__(self):
        self.time_datas = []
        # 能量
        self.kinetic_energy_datas = []
        self.potential_energy_datas = []
        self.total_energy_datas = []
        # 位置
        self.jump_height_datas = []
        self.hip_pos_sensor_datas = []
        # 速度
        self.hip_vel_sensor_datas = []
        self.velocity_sensor_datas = []
        # 加速度
        self.acceleration_sensor_datas = []
        # 力
        self.ground_force_x_sensor_datas = []
        self.ground_force_y_sensor_datas = []
        self.ground_force_z_sensor_datas = []
        # 其他
        self.spring_length_datas = []   # 弹簧长度
        self.theta_datas = []  # 小腿leg连杆与地面的夹角
        self.site_imu_pos_datas = []
        # 质心位置、速度、加速度
        self.cm_pos_x_datas = []
        self.cm_pos_y_datas = []
        self.cm_pos_z_datas = []
        self.cm_vel_x_datas = []
        self.cm_vel_y_datas = []
        self.cm_vel_z_datas = []
        self.cm_acc_x_datas = []
        self.cm_acc_y_datas = []
        self.cm_acc_z_datas = []
        
    def clear_data(self):
        """
        清空数据
        """
        for attr in dir(self):
            if attr.endswith('_datas') or attr.endswith('times'):
                setattr(self, attr, [])
    
    def save_data(self, save_dir:str = "./temp/sensor_data/"):
        """
        保存数据，默认保存在temp文件夹
        """
        import csv  
        now = datetime.now()
        date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
        print("当前时间为",date_time)
        filename = save_dir + "sensor_data_" + date_time + ".csv"
        with open(filename, mode='w', newline='') as file:
            # 重构这个 减少层数
            writer = csv.writer(file)
            headers = [attr.replace('_datas', '').replace('_', ' ').title() for attr in dir(self) if attr.endswith('_datas')]
            writer.writerow(headers)
            for i in range(len(self.time_datas)):
                row = []
                for attr in dir(self):
                    if attr.endswith('_datas'):
                        data = getattr(self, attr)
                        if i < len(data):
                            row.append(data[i])
                        else:
                            row.append(None)
                writer.writerow(row)

    def plot_data(self):
        row = 6
        column = 1
        plt.figure()
        plt.clf()
        plt.grid(True, linewidth=0.5)
        
        plt.subplot(row, column, 1)
        plt.grid(True)
        plt.plot(self.time_datas, self.kinetic_energy_datas, label='Kinetic Energy')
        plt.plot(self.time_datas, self.potential_energy_datas, label='Potential Energy')
        plt.plot(self.time_datas, self.total_energy_datas, label='Total Energy')
        plt.xlabel('Time')
        plt.ylabel('Energy')
        plt.legend()
        
        plt.subplot(row, column, 2)
        plt.grid(True)
        plt.plot(self.time_datas, self.jump_height_datas, label='foot_height')
        plt.xlabel('Time')
        plt.ylabel('position')
        plt.legend()
        plt.draw()
        
        plt.subplot(row, column, 3)
        plt.grid(True)
        plt.plot(self.time_datas, self.velocity_sensor_datas, label='velocity')
        plt.xlabel('Time')
        plt.ylabel('velocity')
        plt.legend()
        plt.draw()
        
        plt.subplot(row, column, 4)
        plt.grid(True)
        plt.plot(self.time_datas, self.acceleration_sensor_datas, label='acceleration')
        plt.xlabel('Time')
        plt.ylabel('acceleration')
        plt.legend()
        plt.draw()
        
        plt.subplot(row, column, 5)
        plt.grid(True)
        plt.plot(self.time_datas, self.hip_pos_sensor_datas, label='hip_angle')
        plt.plot(self.time_datas, self.theta_datas, label='foot_angle')
        plt.xlabel('Time')
        plt.ylabel('theta')
        plt.legend()
        plt.draw()
        
        # plt.subplot(row, column, 6)
        # plt.plot(self.time_datas, self.theta_datas, label='theta')
        # plt.plot(self.time_datas, self.hip_pos_sensor_datas, label='hip_pos_sensor')
        # plt.xlabel('Time')
        # plt.ylabel('angle')
        # plt.legend()
        # plt.draw()
        
        plt.subplot(row, column, 6)
        plt.plot(self.time_datas, self.cm_pos_z_datas, label='cm_pos_z')
        plt.xlabel('Time')
        plt.ylabel('z')
        plt.legend()
        plt.draw()
        
        plt.pause(0.0001)     

    def save_image(self, save_dir:str = "./temp/image/"):
        row = 6
        column = 1
        plt.figure(figsize=(5, 11))
        plt.clf()
        # plt.figure(2)
        plt.subplot(row, column, 1)
        plt.grid(True, linewidth=0.5)
        plt.plot(self.time_datas, self.kinetic_energy_datas, label='Kinetic Energy')
        plt.plot(self.time_datas, self.potential_energy_datas, label='Potential Energy')
        plt.plot(self.time_datas, self.total_energy_datas, label='Total Energy')
        plt.xlabel('Time')
        plt.ylabel('Energy')
        plt.legend()
        
        plt.subplot(row, column, 2)
        plt.grid(True, linewidth=0.5)
        plt.plot(self.time_datas, self.jump_height_datas, label='foot_height')
        plt.plot(self.time_datas, self.cm_pos_z_datas, label='cm_pos_z')
        plt.xlabel('Time')
        plt.ylabel('position')
        plt.legend()
        
        plt.subplot(row, column, 3)
        plt.grid(True, linewidth=0.5)
        # plt.plot(self.time_datas, self.velocity_sensor_datas, label='velocity')
        plt.plot(self.time_datas, self.cm_vel_z_datas, label='cm_velocity')
        plt.xlabel('Time')
        plt.ylabel('velocity')
        plt.legend()
        
        plt.subplot(row, column, 4)
        plt.grid(True, linewidth=0.5)
        # plt.plot(self.time_datas, self.acceleration_sensor_datas, label='acceleration')
        plt.plot(self.time_datas, self.cm_acc_z_datas, label='cm_acceleration')
        plt.xlabel('Time')
        plt.ylabel('acceleration')
        plt.legend()
        
        # plt.subplot(row, column, 5)
        # plt.grid(True, linewidth=0.5)
        # plt.plot(self.time_datas, self.ground_force_x_sensor_datas, label='ground_force_x')
        # plt.plot(self.time_datas, self.ground_force_y_sensor_datas, label='ground_force_y')
        # plt.plot(self.time_datas, self.ground_force_z_sensor_datas, label='ground_force_z')
        # plt.xlabel('Time')
        # plt.ylabel('ground_force')
        # plt.legend()
        # plt.subplot(row, column, 5)
        # plt.grid(True)
        # plt.plot(self.time_datas, self.hip_pos_sensor_datas, label='hip_angle')
        # plt.plot(self.time_datas, self.theta_datas, label='foot_angle')
        # plt.xlabel('Time')
        # plt.ylabel('theta')
        # plt.legend()
        # plt.draw()
        x = self.cm_pos_z_datas.copy()
        y = self.cm_acc_z_datas.copy()
        indices_to_exclude = []
        for index, item in enumerate(y):
            if item is None:
                indices_to_exclude.append(index)
        x = np.delete(x, indices_to_exclude)
        y = np.delete(y, indices_to_exclude)

        plt.subplot(row, column, 6)
        plt.grid(True, linewidth=0.5)
        plt.plot(x, y, label='acc')
        plt.xlabel('cm_pos_z')
        plt.ylabel('acc')
        plt.legend()
        plt.draw()
        
        # plt.subplot(row, column, 6)
        # plt.plot(self.time_datas, self.theta_datas, label='theta')
        # plt.plot(self.time_datas, self.hip_pos_sensor_datas, label='hip_pos_sensor')
        # plt.xlabel('Time')
        # plt.ylabel('angle')
        # plt.legend()
        x = self.cm_pos_z_datas.copy()
        y = self.cm_vel_z_datas.copy()
        indices_to_exclude = []
        for index, item in enumerate(y):
            if item is None:
                indices_to_exclude.append(index)
        x = np.delete(x, indices_to_exclude)
        y = np.delete(y, indices_to_exclude)

        plt.subplot(row, column, 5)
        plt.grid(True, linewidth=0.5)
        plt.plot(x, y, label='velocity')
        plt.xlabel('cm_pos_z')
        plt.ylabel('velocity')
        plt.legend()
        plt.draw()
        plt.tight_layout()
        
        now = datetime.now()
        date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
        filename = save_dir + "data_image_" + date_time + ".png"
        plt.savefig(filename)

class Hopper1(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0  # 停止时间
        self.Hz = 20
        self.is_plot_data = True
        self.is_save_image = True
        self.is_save_data = True
        self.is_render = True
        self.fsm = None
        self.is_tqdm = False
        # self.step_no = 0
        # 储存用于画图和分析的数据
        self.hopperdata = HopperData()
        
    def reset(self):
        self.hopperdata.clear_data()
        # self.model.opt.gravity = [0,0,-1]
        mj.mj_resetData(self.model, self.data)
        mj.mj_forward(self.model, self.data)
        self.fsm = FSM_TAKEOFF # 有限状态机（Finite State Machine）
        mj.set_mjcb_control(self.controller)
        if (self.is_render):
            # Init GLFW, create window, make OpenGL context current, request v-sync
            glfw.init()
            self.window = glfw.create_window(1200, 900, "Demo", None, None)
            glfw.make_context_current(self.window)
            glfw.swap_interval(1)
            # initialize visualization data structures
            mj.mjv_defaultCamera(self.cam)
            mj.mjv_defaultOption(self.opt)
            self.scene = mj.MjvScene(self.model, maxgeom=10000)
            self.context = mj.MjrContext(
                self.model, mj.mjtFontScale.mjFONTSCALE_150.value)
            # install GLFW mouse and keyboard callbacks
            glfw.set_key_callback(self.window, self.keyboard)
            glfw.set_cursor_pos_callback(self.window, self.mouse_move)
            glfw.set_mouse_button_callback(self.window, self.mouse_button)
            glfw.set_scroll_callback(self.window, self.scroll)
            glfw.set_key_callback(self.window, self.keyboard)
            # Set camera configuration
            self.cam.azimuth = 89.608063
            self.cam.elevation = -11.588379
            self.cam.distance = 5.0
            self.cam.lookat = np.array([0.0, 0.0, 1.5])
    
    def keyboard(self, window, key, scancode, act, mods):
        """
        绑定键盘回调函数
        """
        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)
            self.fsm = FSM_TAKEOFF # 有限状态机（Finite State Machine）
            plt.clf()
            self.hopperdata.clear_data()
        if act == glfw.PRESS and key == glfw.KEY_ESCAPE: 
            # 默认按escape退出即保存所有数据
            if self.is_save_data:
                self.hopperdata.save_data()
            if self.is_save_image:
                self.hopperdata.save_image()
            model_text_dir = "./temp/model.txt"
            data_text_dir = "./temp/data.txt"
            mj.mj_printModel(self.model, model_text_dir)
            mj.mj_printData(self.model, self.data, data_text_dir)       
            glfw.set_window_should_close(window, True)
        if act == glfw.PRESS and key == glfw.KEY_A: 
            self.hopperdata.clear_data()
            plt.clf()
            pservo_hip_left_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, "pservo_hip_left")
            pservo_hip_right_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, "pservo_hip_right")
            # print('left_hip_roll_id', left_hip_roll_id)
            # 这里的数字的单位是什么
            # self.data.ctrl[pservo_hip_left_id] = -47
            # self.data.ctrl[pservo_hip_right_id] = 47
            # mj.mj_resetDataKeyframe(self.model, self.data, 0)
            self.fsm = FSM_TAKEOFF
            
            # print(self.data.qpos)
        if act == glfw.PRESS and key == glfw.KEY_D: 
            print(self.data.sensordata)
    
    def controller(self, model, data):
        """
        控制关节的行为
        """
        # return
        # 之后不要直接读取位置，而是通过传感器读取位置，给传感器扰动，以符合现实传感器波动导致机器不稳的情况
        # z_foot = data.sensordata[0]
        # body_no = 3
        # z_foot = data.xpos[body_no, 2]
        # vz_torso = data.qvel[1]
        # height = self.data.qpos[8] - self.model.qpos0[8]
        spring_siteid:int = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_TENDON, "tendon1")
        spring_length = self.data.ten_length[spring_siteid]
        left_hip_roll_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, "left_hip_roll")
        right_hip_roll_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, "right_hip_roll")
        mj.mj_energyPos(model, data)
        mj.mj_energyVel(model, data)
        return
        during_time = self.data.time - self.start_time

        if during_time < 1 :
            print('0')
            self.data.ctrl[left_hip_roll_id] = 0
            self.data.ctrl[right_hip_roll_id] = 0
        if during_time < 2 and during_time >= 1:
            print('1')
            tmp = during_time*2-2
            self.data.ctrl[left_hip_roll_id] = -tmp
            self.data.ctrl[right_hip_roll_id] = tmp

        if during_time > 2:
            print('2')
            self.data.ctrl[left_hip_roll_id] = 0
            self.data.ctrl[right_hip_roll_id] = 0
            
            
    def bind_data(self):
        """
        收集数据
        """
        if self.data.time < 0.95:
            self.height_original = self.data.qpos[13]
            return

        spring_siteid:int = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_TENDON, "tendon1")
        spring_length = self.data.ten_length[spring_siteid]

        height = self.data.qpos[13] - self.height_original  # 机构最低点距离地面的高度
        self.hopperdata.time_datas.append(self.data.time)
        self.hopperdata.kinetic_energy_datas.append(self.data.energy[1])
        self.hopperdata.potential_energy_datas.append(self.data.energy[0])
        self.hopperdata.total_energy_datas.append(self.data.energy[0] + self.data.energy[1])
        self.hopperdata.spring_length_datas.append(spring_length)
        self.hopperdata.jump_height_datas.append(height)
        self.hopperdata.hip_pos_sensor_datas.append(self.data.sensordata[0]*180/3.1415)
        self.hopperdata.hip_vel_sensor_datas.append(self.data.sensordata[1])
        self.hopperdata.acceleration_sensor_datas.append(self.data.sensordata[4])
        self.hopperdata.velocity_sensor_datas.append(self.data.sensordata[7])
        imu_siteid:int = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, "imu")
        self.hopperdata.site_imu_pos_datas.append(self.data.site_xpos[imu_siteid][2])

        total_mass = sum(self.model.body_mass)
        cm_pos_x = 0
        cm_pos_y = 0
        cm_pos_z = 0
        for i in range(self.model.nbody):
            cm_pos_x += self.data.geom_xpos[i, 0] * self.model.body_mass[i]
            cm_pos_y += self.data.geom_xpos[i, 1] * self.model.body_mass[i]
            cm_pos_z += self.data.geom_xpos[i, 2] * self.model.body_mass[i]
        cm_pos_x /= total_mass
        cm_pos_y /= total_mass
        cm_pos_z /= total_mass
        self.hopperdata.cm_pos_x_datas.append(cm_pos_x)
        self.hopperdata.cm_pos_y_datas.append(cm_pos_y)
        self.hopperdata.cm_pos_z_datas.append(cm_pos_z)

        if len(self.hopperdata.cm_pos_x_datas) < 2:
            self.hopperdata.cm_vel_x_datas.append(None)
            self.hopperdata.cm_vel_y_datas.append(None)
            self.hopperdata.cm_vel_z_datas.append(None)
        else:
            delta_time = self.hopperdata.time_datas[-1] - self.hopperdata.time_datas[-2]
            cm_vel_x = (self.hopperdata.cm_pos_x_datas[-1] - self.hopperdata.cm_pos_x_datas[-2])/delta_time
            cm_vel_y = (self.hopperdata.cm_pos_y_datas[-1] - self.hopperdata.cm_pos_y_datas[-2])/delta_time
            cm_vel_z = (self.hopperdata.cm_pos_z_datas[-1] - self.hopperdata.cm_pos_z_datas[-2])/delta_time

            self.hopperdata.cm_vel_x_datas.append(cm_vel_x)
            self.hopperdata.cm_vel_y_datas.append(cm_vel_y)
            self.hopperdata.cm_vel_z_datas.append(cm_vel_z)

        if len(self.hopperdata.cm_pos_x_datas) < 3:
            self.hopperdata.cm_acc_x_datas.append(None)
            self.hopperdata.cm_acc_y_datas.append(None)
            self.hopperdata.cm_acc_z_datas.append(None)
        else:
            delta_time = self.hopperdata.time_datas[-1] - self.hopperdata.time_datas[-2]
            cm_acc_x = (self.hopperdata.cm_vel_x_datas[-1] - self.hopperdata.cm_vel_x_datas[-2])/delta_time
            cm_acc_y = (self.hopperdata.cm_vel_y_datas[-1] - self.hopperdata.cm_vel_y_datas[-2])/delta_time
            cm_acc_z = (self.hopperdata.cm_vel_z_datas[-1] - self.hopperdata.cm_vel_z_datas[-2])/delta_time
            self.hopperdata.cm_acc_x_datas.append(cm_acc_x)
            self.hopperdata.cm_acc_y_datas.append(cm_acc_y)
            self.hopperdata.cm_acc_z_datas.append(cm_acc_z)

        q = self.data.xquat[5]
        if np.array_equal(q, np.array([0, 0, 0, 0])):
            self.hopperdata.theta_datas.append(None)
            # print("跳过")
        else:
            r = R.from_quat(q)
            # 将旋转矩阵转换为欧拉角
            euler_angles = r.as_euler('zyx', degrees=True)  # 这里假设是绕z、y、x轴的欧拉角
            self.hopperdata.theta_datas.append(euler_angles[1]+90)            
        id=0
        result = np.zeros((6,1), dtype=np.float64)
        mj.mj_contactForce(self.model, self.data, id, result);
        self.hopperdata.ground_force_x_sensor_datas.append(result[0,0])
        self.hopperdata.ground_force_y_sensor_datas.append(result[1,0])
        self.hopperdata.ground_force_z_sensor_datas.append(result[2,0])

    def simulate(self):
        self.start_time = self.data.time
        if self.is_tqdm:
            total_steps = 100  # 总的步骤数
            progress_bar = tqdm(total=total_steps, leave=False)  # 创建进度条对象
            time_interval = self.simend / total_steps  # 计算时间间隔
            current_progress = 0  # 当前进度
        if (self.is_render):
            while not glfw.window_should_close(self.window) and self.data.time < self.simend:
                self.bind_data()    # 收集数据
                current_simstart = self.data.time   # 当前一帧的开始时间
                while (self.data.time - current_simstart < 1.0/self.Hz):
                    mj.mj_step(self.model, self.data)                      
                # get framebuffer viewport
                viewport_width, viewport_height = glfw.get_framebuffer_size(
                    self.window)
                viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

                # Update scene and render
                self.cam.lookat[0] = self.data.qpos[0]
                mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                                mj.mjtCatBit.mjCAT_ALL.value, self.scene)
                mj.mjr_render(viewport, self.scene, self.context)
                # swap OpenGL buffers (blocking call due to v-sync)
                glfw.swap_buffers(self.window)
                # process pending GUI events, call GLFW callbacks
                glfw.poll_events()
                if self.is_plot_data and self.data.time> 0.95:
                    self.hopperdata.plot_data()
                if self.is_tqdm:     
                    current_progress = int(self.data.time / time_interval)  # 计算当前进度
                    progress_bar.update(current_progress - progress_bar.n)  # 更新进度条
            glfw.terminate()
            return True
        else:
            while self.data.time < self.simend:
                self.bind_data()    # 收集数据
                current_simstart = self.data.time   # 当前一帧的开始时间
                while (self.data.time - current_simstart < 1.0/self.Hz):
                    mj.mj_step(self.model, self.data)
                # print(self.data.energy[1])   
                if self.is_plot_data:
                    self.hopperdata.plot_data()
                if self.is_tqdm:
                    current_progress = int(self.data.time / time_interval)  # 计算当前进度
                    progress_bar.update(current_progress - progress_bar.n)  # 更新进度条
            if self.is_save_image:
                self.hopperdata.save_image()
            if self.is_save_data:
                self.hopperdata.save_data()
                model_text_dir = "./temp/model.txt"
                data_text_dir = "./temp/data.txt"
                mj.mj_printModel(self.model, model_text_dir)
                mj.mj_printData(self.model, self.data, data_text_dir)
            return True
    
    def initial_state(self):
        # self.model.body_pos[1,2] = 3.5
        # self.cam.lookat[0] = self.data.qpos[0]
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        mj.mj_step(self.model, self.data)
        # 可以通过 self.model.body_pos 来改变物体的初始位置
        # print(f"body_pos\n{self.model.body_pos}")
        print(self.data.qpos)
        
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
        
    def set_position_servo(self, actuator_no, kp):
        self.model.actuator_gainprm[actuator_no, 0] = kp
        self.model.actuator_biasprm[actuator_no, 1] = -kp
    
    def set_velocity_servo(self, actuator_no, kv):
        self.model.actuator_gainprm[actuator_no, 0] = kv
        self.model.actuator_biasprm[actuator_no, 2] = -kv

def main():
    # xml_path = "./xml/hopper1/scene.xml"
    # xml_path = "./xml/hopper1/hopper4.xml"
    # xml_path = "./xml/hopper1/hopper_longSideBar_small.xml"
    xml_path = "./xml/hopper1/hopper_longSideBar.xml"
    sim = Hopper1(xml_path)
    sim.simend = 5
    sim.Hz = 50
    sim.is_plot_data = False
    # sim.model.tendon_stiffness[0] = 50
    # sim.model.tendon_lengthspring[1] = [1, 1]
    # tmp = 2
    # sim.model.tendon_lengthspring[1] = [tmp, tmp]
    # sim.model.tendon_lengthspring[2] = [tmp, tmp]
    # print(sim.model.tendon_lengthspring[1, 0])
    # sim.is_save_image = False
    # sim.is_save_data = False
    # sim.is_render = False
    # sim.is_tqdm = True
    sim.reset()
    sim.simulate()
    # print(sim.model.tendon_stiffness)
    
    # sim.initial_state()

if __name__ == "__main__":
    main()

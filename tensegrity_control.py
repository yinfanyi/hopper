import mujoco as mj
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw

import numpy as np


xml_path = "./xml/TT_test.xml"

class Tensegrity(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0  # 停止时间
        self.Hz = 20
        self.is_plot_data = True
        self.is_save_image = True
        self.is_save_data = True
        self.is_render = True
        self.fsm = None
        
    def reset(self):
        # self.model.opt.gravity = [0,0,-1]
        mj.mj_resetData(self.model, self.data)
        mj.mj_forward(self.model, self.data)
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
            self.hopperdata.clear_data()
        if act == glfw.PRESS and key == glfw.KEY_ESCAPE: 
            glfw.set_window_should_close(window, True)
         
    def simulate(self):
        self.start_time = self.data.time

        if (self.is_render):
            while not glfw.window_should_close(self.window) and self.data.time < self.simend:
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
            glfw.terminate()
            return True
        else:
            while self.data.time < self.simend:
                current_simstart = self.data.time   # 当前一帧的开始时间
                while (self.data.time - current_simstart < 1.0/self.Hz):
                    mj.mj_step(self.model, self.data)
                if self.is_plot_data:
                    self.hopperdata.plot_data()
            return True
    
    def initial_state(self):
        # self.model.body_pos[1,2] = 3.5
        # self.cam.lookat[0] = self.data.qpos[0]
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        mj.mj_step(self.model, self.data)
        
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

sim = Tensegrity(xml_path)
sim.simend = 20
sim.Hz = 100
sim.reset()
sim.simulate()
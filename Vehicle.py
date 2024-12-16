import numpy as np
class Car:
    def __init__(self, lx, ly, vx, vy, ax, ay, lane, speed,v_max=float('inf'),v_min=0): #这里包含“self”和等式右边的参数
        self.loc_x = lx
        self.loc_y = ly
        self.v_x = vx
        self.v_y = vy
        self.a_x = ax
        self.a_y = ay
        self.lane = lane
        self.just_changed_lane = False
        self.speed = speed
        self.v_max = v_max
        self.v_min = v_min
        self.speeds = []
        self.trajectory = []
        self.preparingBrake = 0.5


    def update_loc(self,time_step):
        self.loc_x += self.v_x * time_step
        self.loc_y += self.v_y * time_step

    def update_v(self, a_x, a_y,time_step):  #这个自己函数里边写的参数，为什么要写？和全局有什么区别?
        self.a_x = a_x
        self.a_y = a_y
        self.v_x += self.a_x * time_step
        new_v = self.v_y + self.a_y * time_step
        self.v_y = min(self.v_max, new_v)
    
    def get_speed(self):
        return np.sqrt(self.v_x**2 + self.v_y**2)
    
    def update_trajectory(self, time, lx, ly):
        self.trajectory.append((time, lx, ly))
        current_speed = self.get_speed()
        self.speeds.append((time, current_speed))
        self.speed = current_speed  # 记录当前速度 这句话起的作用是什么？
    
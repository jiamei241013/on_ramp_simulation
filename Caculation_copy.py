import numpy as np
class Caculator:
    def __init__(self,tao,s_r,x_xing,V_max,c1,c2,c3,line_cen_HOV,k1,k2,time_step):
        self.tao = tao
        self.s_r = s_r
        self.x_xing = x_xing
        self.V_max = V_max
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.line_cen_HOV = line_cen_HOV
        self.k1 = k1
        self.k2 = k2
        self.time_step = time_step
        
    # 力的计算函数
    def cal_frik(self,car_i, car_k):
        v_i = np.array([[car_i.v_x], [car_i.v_y]])
        v_k = np.array([[car_k.v_x], [car_k.v_y]])
        v_i_t = np.linalg.norm(v_i)
        q_v = (v_i_t * self.tao + self.s_r) / self.x_xing
        Q = np.array([[1, 0], [0, q_v]])
        Q_ni = np.linalg.inv(Q)
        x_i = np.array([[car_i.loc_x], [car_i.loc_y]])
        x_k = np.array([[car_k.loc_x], [car_k.loc_y]])
        r_ik = np.dot(Q_ni, x_k - x_i)
        r_ik_dw = r_ik / np.linalg.norm(r_ik) if np.linalg.norm(r_ik) != 0 else r_ik
        v_i_det = np.dot(np.transpose(np.dot(Q_ni, v_k - v_i)), r_ik_dw)
        f_r_ik = np.dot(Q, r_ik_dw) * min(0, self.c2 * v_i_det + (np.linalg.norm(r_ik) - self.x_xing) * self.c3)
        return f_r_ik

    def cal_faik(self,car_i):
        vmax_det = self.V_max - car_i.v_y
        f_a_i = np.array([[0], [self.c1 * vmax_det]])  # 纵向加速度
        return f_a_i

    def cal_flik(self,car_i):
        v_i_x = car_i.v_x
        x_l_det = car_i.loc_x - self.line_cen_HOV
        f_l_i = np.array([[-v_i_x * self.k1 - x_l_det * self.k2], [0]])  # 横向调整力
        return f_l_i

    def cal_fsumik(self,car_i, car_k):
        f_sum_ik = self.cal_faik(car_i) + self.cal_flik(car_i) + self.cal_frik(car_i, car_k)
        return f_sum_ik


    # 车道变换后的放松逻辑
    def relax_after_lane_change(self,car,main_road_vehicles, min_spacing=30,merge_area_start=390,merge_area_end=630):
        if car.just_changed_lane:
            # 检查车辆是否在变道区域内
            # if 390 <= car.loc_y <= 630:
            if car.loc_y >= merge_area_start and car.loc_y <= merge_area_end:
                # 在区域内，立即触发放松逻辑
                for other_car in main_road_vehicles:
                    if other_car != car and other_car.lane == car.lane:
                        distance = np.sqrt((car.loc_x - other_car.loc_x) ** 2 + (car.loc_y - other_car.loc_y) ** 2)
                        # 如果距离大于最小间距，调整车辆位置以保持间距
                        if distance > min_spacing:
                            # 减速但不低于20 m/s
                            car.v_y = max(11.5, car.v_y - 0.5)
                        else:
                            car.v_y = min(self.V_max, car.v_y +0.5)
            # 如果车辆已经通过了放松区域，逐渐加速回到最大速度
            if car.loc_y > 500:
                car.v_y = min(self.V_max, car.v_y + 0.5)  # 加速但不超过V_max m/s
            car.just_changed_lane = False  # 重置标志位

    def update_parameter(self,main_road_vehicles,step,
                         isExecution=False,brakeTime=None,brakePosition=None,
                         merge_area_start=390,merge_area_end=630):
        # 数据更新
        for i, car in enumerate(main_road_vehicles[:]):
            # 是否靠近匝道
            if car.loc_y > merge_area_start and car.loc_y < merge_area_end and car.lane == 0:
                car.v_y = max(11.5, car.v_y - 0.5)  # 减速但不低于20 m/s
            elif car.loc_y > merge_area_start and car.loc_y < merge_area_end and car.lane == 1:
                car.v_y = max(11.5, car.v_y - 0.5)  # 减速但不低于11.5 m/s
                car.lane = 0
                car.just_changed_lane = True
                self.relax_after_lane_change(car,main_road_vehicles=main_road_vehicles)

            # 更新车辆的速度和位置
            for j in range(len(main_road_vehicles)):
                if i != j:  # 避免与自身比较
                    car_j = main_road_vehicles[j]
                    # 计算两车之间的距离
                    distance = np.sqrt((car.loc_x - car_j.loc_x) ** 2 + (car.loc_y - car_j.loc_y) ** 2)
                    # 如果距离小于安全距离，则增加斥力
                    if distance < 100:  # 假设安全距离为5米
                        force = self.cal_fsumik(car, car_j)
                        a_x = force[0][0]
                        a_y = force[1][0]
                        car.update_v(a_x, a_y, self.time_step)

           
            # 更新车辆的位置
            car.update_loc(self.time_step)

            # 记录车辆位置
            if car.lane == 0:
                car.update_trajectory(step*self.time_step, car.loc_x, car.loc_y)
            else:
                car.update_trajectory(step*self.time_step, None, None)

        return main_road_vehicles,isExecution,brakeTime
    



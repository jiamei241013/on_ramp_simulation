import math
# simulation、Caculator、Vision都代表类名
from Simulation import simulation
from Caculation_copy import Caculator
from Draw import Vision

if __name__ == "__main__":

    # 常量定义
    k2 = 1 / 4
    k1 = 2 * math.sqrt(k2)
    x_xing = 2
    c1 = 1 / 10
    c2 = 33 / 64
    c3 = 9 / 64
    tao = 8 / 10
    s_r = 196 / 9
    V_max = 22.5
    line_cen_HOV = -2
    delta_r = 2.9  # 排斥力差异阈值[m/s^2]
    d_t = 15  # 速度降低阈值[m/s]
    time_step = 0.1  # 时间步长(与精度相关)
    car_width = 2  # 车辆宽度
    lane_width = 4  # 车道宽度
    # 初始化道路信息
    main_road_vehicles = []
    # 设置模拟参数
    num_steps = 2000
    total_time = num_steps * time_step  # 模拟的总时间

    # 创建模拟器，  使用类里边的函数之前需要先进行定义，不在类里边的可以直接调用函数;进行赋值的话就可以直接用类，并且将形参给定具体的数值
    simulater = simulation()
    vision = Vision()
    # 创建计算器，内部形参需要包括他所有的属性
    caculator = Caculator(tao,s_r,x_xing,V_max,c1,c2,c3,line_cen_HOV,k1,k2,time_step)   
    # 主循环
    # 模拟车辆运动
    for step in range(num_steps):
        current_time = step * time_step  # 当前模拟时间
        # 主路来车
        if current_time % 5 == 0:
           
           main_car = simulater.createVehicle(lx=0, ly=0, vx=0, vy=0, ax=0, ay=0, lane=0, speed=0,v_max=V_max)
           main_road_vehicles.append(main_car)
        # 辅路来车
        if current_time % 15 == 0:
            ramp_car = simulater.createVehicle(lx=4, ly=0, vx=0, vy=0, ax=0, ay=0, lane=1, speed=0,v_max=V_max)
            main_road_vehicles.append(ramp_car)
        # 更新车辆参数
        main_road_vehicles,isExecution,brakeTime = caculator.update_parameter(main_road_vehicles=main_road_vehicles,step=step)
          
    vision.visible_1(main_road_vehicles,V_max,total_time)  

        
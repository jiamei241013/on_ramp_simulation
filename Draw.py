import matplotlib.pyplot as plt
import matplotlib
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import numpy as np

class Vision():
    def __init__(self):
        pass
    def visible_1(self,main_road_vehicles,V_max,total_time):
        # 可视化 创建图形和轴
        fig, ax = plt.subplots(figsize=(10, 6))

        # 创建一个归一化对象，用于将速度值映射到颜色映射的范围内
        norm = Normalize(vmin=0, vmax=V_max)

        # 创建一个颜色映射对象，例如使用viridis颜色映射
        cmap = plt.cm.viridis

        for car in main_road_vehicles:
            if hasattr(car, 'trajectory') and car.trajectory: # 检查车辆是否有轨迹
                times = [t[0] for t in car.trajectory]
                y_coords = [t[2] for t in car.trajectory]
                speeds = [v[1] for v in car.speeds] # 获取记录的速度数据

                # 绘制车辆轨迹，使用速度数据为轨迹着色
                segments = np.array([(times[i], y_coords[i], times[i+1], y_coords[i+1]) for i in range(len(times)-1)]).reshape(-1, 2, 2)
                lc = LineCollection(segments, cmap=cmap, norm=norm)
                lc.set_array(speeds[:-1]) # 确保速度数组与线段数量匹配
                lc.set_linewidth(2)
                ax.add_collection(lc)

                # 创建LineCollection对象
                lc = LineCollection(segments, cmap=cmap, norm=norm)
                lc.set_array(np.array(speeds))
                lc.set_linewidth(2) # 设置线宽
                ax.add_collection(lc)

        # 设置坐标轴的范围
        ax.set_xlim(0, total_time)
        ax.set_ylim(0, 5000)

        # 设置坐标轴的标签
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Y Position (m)')

        # 显示图形
        plt.colorbar(lc, ax=ax) # 显示颜色条
        plt.show()


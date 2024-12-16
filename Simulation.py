from Vehicle import Car

class simulation:
    def __init__(self):
        pass
    
    # def createRoad():
    #     road = []
    #     return road
    
    def createVehicle(self,
            # 车辆相关参数
            lx, ly, vx, vy, ax, ay, lane, speed,v_max=float('inf'),v_min=0
            ):
        car = Car(lx, ly, vx, vy, ax, ay, lane, speed,v_max,v_min)
        return car
from .djitellopy import Tello, TelloSwarm, enforce_types
from MotionPlanner import MotionPlanner

@enforce_types
class Drones:
    def __init__(self):
        self.drone_data = {}
    
    def add_drone(self, name, ip):
        id = TelloSwarm.add_drone(ip)
        drone = Drone(name, id, "192.168.10.1")
        self.drone_data[name] = drone


class Drone(Tello):
    """
    Creates a unique Tello instance for each drone which can be accessed through Drones
    """
    def __init__(self, name, id, host):
        super(Drone, self).__init__(host=host)
        self.name = name
        self.tello = None
        self.state = State()
        self.motion_planner = MotionPlanner()
    
    @staticmethod
    def connect():
        pass

    def send_v():
        pass


class State:
    def __init__(self, x = 0.0, y=0.0, z =0.0, yaw=0.0):
        """"
        Store states of Tello Drone in a state object

        Arguments:
            x, y, z, yaw
        Methods:
            update(Updates state of the drone to current state)
        """
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.v = [0.0,0.0,0.0,0.0]

        self.battery_level = None
        self.temp_level = None

        self.is_flying = False
        self.is_hovering = False

        

    def update(self,x = None,y = None,z = None,yaw = None, battery_level = None, temp_level = None):
        if not x == None:
            self.x = x

        if not y == None:
            self.y = y

        if not z == None:
            self.z = z

        if not yaw == None:
            self.yaw = yaw

        if not battery_level == None:
            self.battery_level = battery_level

        if not temp_level == None:
            self.temp_level = temp_level

    def update_velocities(self, x, y, z, yaw):
        self.v = [x,y,z,yaw]

drones = Drones()
for i in range(5):
    drones.add_drone(i)

for i in range(5):
    print(drones.drone_data[i])
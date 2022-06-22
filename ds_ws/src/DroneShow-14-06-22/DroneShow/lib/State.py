from .tellopy import Tello, TelloSwarm, enforce_types
from MotionPlanner import MotionPlanner

@enforce_types
class Swarm:
    def __init__(self):
        self.data = {}
        self.ips = []
        self.id = 0
        self.swarm_controller = None
    
    def add_drone(self, name, ip):
        self.ips.append[ip]
        drone = Drone(name, self.id, ip)
        self.data[name] = drone
        self.id += 1

    def connect(self):
        #self.swarm_controller = TelloSwarm.fromIps(self.ips)
        print(self.ips)

    def __add_drone(self, name, ip):
        id = TelloSwarm.add_drone(ip)
        drone = Drone(name, id, ip)
        self.data[name] = drone

    def __connect(self):
        self.swarm_controller = TelloSwarm.start_swarm()


class Drone():
    """
    Creates a unique Tello instance for each drone which can be accessed through Drones
    """
    def __init__(self, name, id, host):
        self.name = name
        self.id = id
        self.host = host
        self.state = State()
        self.motion_planner = MotionPlanner()

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
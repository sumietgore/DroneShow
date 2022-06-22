from lib.tellopy import Tello, TelloSwarm
from lib.MotionPlanner import MotionPlanner

class Swarm:
    DRONE_DATA = {}
    DRONE_V_DATA = []
    """
    Creates a swarm of tello drones
    """
    def __init__(self):
        self.data = {}
        self.v_data = []
        self.ips = []
        self.id = 0

        self.swarm_controller = None

        self.swarm_connected = False
    
    def add_drone(self, name, ip, topic_name = None):
        self.ips.append(ip)

        id = self.id

        drone = Drone(name, self.id, ip, topic_name=topic_name)
        self.data[name] = drone
        self.id += 1
        return id

    def connect(self):
        if len(self.ips) == self.id:

            if not len(self.v_data) == self.id:

                for i in range(self.id - len(self.v_data)):
                    self.v_data.append([0,0,0,0])

                self.swarm_controller = TelloSwarm.fromIps(self.ips)
                self.swarm_controller.connect()
                print(self.id)
                self.swarm_connected = True
        else:
            print("Error in parsing drone ips")


    def takeoff(self):
        if self.swarm_connected:
            self.swarm_controller.takeoff()
        else:
            print("Swarm not connected")

    def check_yaw(self):
        pass

    def check_safety_net(self):
        pass

    def controller(self):
        pass

class Drone():
    """
    Creates a unique Tello instance for each drone which can be accessed through Drones
    """
    def __init__(self, name, id, host, topic_name = None):
        self.name = name
        self.id = id
        self.host = host
        self.locale_topic_name = topic_name
        self.state = State()
        self.motion_planner = None

class State:
    def __init__(self, x = 0.0, y=0.0, z =0.0, yaw=0.0, battery_level = None, temp_level = None):
        """"
        Store states of Tello Drone in a state object

        Arguments:
            x, y, z, yaw, battery_level, temp_level
        Methods:
            update : Updates state of the drone to current state
        """
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.v = [0.0,0.0,0.0,0.0]

        self.battery_level = battery_level
        self.temp_level = temp_level

        self.is_flying = False
        self.is_hovering = False


    def update(self,x = None,y = None,z = None,yaw = None, battery_level = None, temp_level = None):
        """
        Update state of the drone to a new state

        Arguments:
            x : x position
            y : y position
            z : z poistion
            yaw : yaw in degrees
            battery_level = battery level of the drone
            temp_level = temperature of the drone
        """
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

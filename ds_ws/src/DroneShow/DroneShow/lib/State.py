from djitellopy import Tello

class Drones:
    def __init__(self):
        self.drone_data = {}
    
    def add_drone(self, name):
        drone = Drone(name, "192.168.10.1")
        self.drone_data[name] = drone


class Drone(Tello):
    """
    Creates a unique Tello instance for each drone which can be accessed through Drones
    """
    def __init__(self, name, host):
        super(Drone, self).__init__(host=host)
        self.name = name
        self.state = State()
    
    @staticmethod
    def connect():
        pass

    def send_v()


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

        self.is_flying = False
        self.is_hovering = False

        self.yaw = yaw
        self.v = [0.0,0.0,0.0,0.0]

    def update(self,x,y,z,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def update_velocities(self, x, y, z, yaw):
        self.v = [x,y,z,yaw]

drones = Drones()
for i in range(5):
    drones.add_drone(i)

for i in range(5):
    print(drones.drone_data[i])
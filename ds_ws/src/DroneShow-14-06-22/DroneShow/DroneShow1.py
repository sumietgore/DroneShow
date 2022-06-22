from tkinter.font import nametofont

from click import password_option
from lib.WaypointGenerator import WaypointGenerator
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from lib.tellopy import Tello
from lib.MotionPlanner import MotionPlanner
from lib.ShowController import ControllerWindow
from lib.Swarm import Swarm
import time
from threading import Thread
import logging

import math

Tello.LOGGER.setLevel(logging.WARN)
 
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class Localisation(Node):
    def __init__(self, controller_window=None):
        super().__init__('Localisation')

        self.callbacks = {}
        self.subscribers = {}

        print("asd1")

        for name in Swarm.DRONE_DATA:
            if Swarm.DRONE_DATA[name]['subscriber'] == None and Swarm.DRONE_DATA[name]['callback'] == None and not Swarm.DRONE_DATA[name]['topic'] == None:
                cb = self.make_callback(name)
                #topic='/drone1_cap/pose'
                #print(type(topic))
                topic = str(Swarm.DRONE_DATA[name]['topic'])
                #print(type(topic))
                #print(Swarm.DRONE_DATA[name])
                self.callbacks[topic] = cb
                self.subscribers[topic] = self.create_subscription(PoseStamped, topic,self.callbacks[topic],10)

    def make_callback(self, topic_name):
        def callback(msg):
            print(msg)
            #print(f'{topic_name} {msg}')
        return callback



class DroneShow():
    def __init__(self):
        self.controller_window = ControllerWindow(file_load_cb=self.file_load_cb, emergency_cb=self.emergency_cb, close_cb=self.close_cb,start_local_cb=self.start_local_cb)
        self.localisation_node = None

        self.star_ros = False

    def emergency_cb(self):
        self.kill = True
        print("Emergency callback")
        self.stop()

    def file_load_cb(self, anim_file_location):
        self.waypoints_x = [20]
        WaypointGenerator(anim_file_location)
        self.controller_window.update_data()
        print(Swarm.DRONE_DATA)
        
    def close_cb(self):
        self.destroyed = True
        self.destroy_node()
        rclpy.shutdown()

    def start_local_cb(self):
        self.star_ros = True

def main(args=None):
    rclpy.init(args=args)
    drone_show = DroneShow()

    while not drone_show.star_ros:
        pass

    print("asd")

    localisation_node = Localisation()
    rclpy.spin(localisation_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
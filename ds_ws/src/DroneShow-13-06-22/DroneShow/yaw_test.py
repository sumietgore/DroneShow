import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from lib.tellopy import Tello
from lib.MotionPlanner import MotionPlanner
import time
from threading import Thread
import numpy as np
import logging
from showcontroller_test import ControllerWindow

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

class Test(Node):
    def __init__(self):
        super().__init__('Motion_Test')

        print("Starting")

        self.cur_x = None
        self.cur_y = None
        self.cur_z = None
        self.is_flying = False
        self.stopped = False
        self.stopped_x = False
        self.stopped_y = False
        self.yaw_aligned = False

        self.controller = ControllerWindow()
        print('asd')

        self.prev_x = None
        self.v_x = 0

        self.counter = 0

        self.waypoints_x = [150, 220, 150]
        self.waypoints_y = [175, 200, 175]
        self.waypoints_x_i = 1
        self.waypoints_y_i = 1

        self.prev_timestamp = None
        self.dt = 10

        self.pose_subscriber = self.create_subscription(PoseStamped,'/vrpn_client_node/drone1_cap/pose',self.pose_callback,1)



    def pose_callback(self, msg):

        self.cur_x = round(float(msg.pose.position.x * 100),1)
        self.cur_y = round(float(msg.pose.position.y * 100),1)
        self.cur_z = round(float(msg.pose.position.z * 100),1)

        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        self.yaw = round(math.degrees(yaw),1)
        #print(self.yaw)
        if self.controller.initialised:
            self.controller.update(self.cur_x, self.cur_y, self.cur_z, self.yaw)

        #if not self.stopped:
        #print(f'x : {self.cur_x} y : {self.cur_y} z : {self.cur_z} roll : {math.degrees(roll)} pitch : {math.degrees(pitch)} yaw : {math.degrees(yaw})')
        #print(f'v_x : {round(self.v_x,0)} x : {round(self.cur_x,2)} y : {round(self.cur_y,2)} z : {round(self.cur_z,2)} yaw : {round(math.degrees(yaw),1)}', end='\r')
        #print(f'{self.v_x}',end='\r')

def main(args=None):
    rclpy.init(args=args)

    test = Test()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
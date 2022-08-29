from calendar import TUESDAY
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

        self.in_position_x = False
        self.in_position_y = False

        self.prev_x = None
        self.v_x = 0

        self.motion_planner = None
        self.motion_planned = False

        self.counter = 0

        self.waypoints_x = [20, 20, 120, 120, 20, 150]
        self.waypoints_y = [100, 200, 200, 100, 100, 150]
        self.waypoints_x_i = 1
        self.waypoints_y_i = 1

        self.prev_timestamp = None
        self.dt = 0

        self.pose_subscriber = self.create_subscription(PoseStamped,'/vrpn_client_node/drone1_cap/pose',self.pose_callback,1)
        self.stop_subscriber = self.create_subscription(String, "/stop",self.stop_callback, 1)

        self.control_thread = None

    @threaded
    def controller(self):
        while not self.stopped:
            if not self.yaw_aligned:
                print(f'yaw : {self.yaw}', end='\r')
            if self.yaw_aligned and self.motion_planned:
            #print(f'x : {self.cur_x} y : {self.cur_y} z : {self.cur_z}')
                print(f'x : {round(self.cur_x,2)} des_x {self.motion_planner.desired_x} y : {round(self.cur_y,2)} z : {round(self.cur_z,2)} yaw : {self.yaw}' , end='\r')

                #Code for safety net            
                if self.cur_x > 275 or self.cur_y > 275 or self.cur_z > 210:
                    print(f'Safety net activated {self.cur_x} {self.cur_y} {self.cur_z}')
                    self.stopped_x = True
                    self.stopped_y = True
                    self.tello.stop()
                    self.tello.send_rc_control(0,0,0,0)
                    time.sleep(1)
                    self.tello.land()
                    self.tello.end()

                if self.prev_timestamp == None:
                    self.prev_timestamp = time.perf_counter()
                
                #Control for x-axis
                if abs(self.motion_planner.desired_x - self.cur_x) > 5:
                    self.in_position_x = False
                    self.stopped_x = False
                    if self.motion_planner.desired_x > self.cur_x:
                        v_x = 5
                        if self.motion_planner.desired_x_vel > v_x:
                            v_x = int(self.motion_planner.desired_x_vel)
                        if int(self.motion_planner.desired_x_vel) == 0:
                            v_x = 10
                        if abs(self.motion_planner.desired_x - self.cur_x) > 10:
                            v_x = int(self.motion_planner.desired_x_vel)
                    else:
                        v_x = -5
                        if self.motion_planner.desired_x_vel < v_x:
                            v_x = -int(self.motion_planner.desired_x_vel)
                        if int(self.motion_planner.desired_x_vel) == 0:
                            v_x = -10
                        if abs(self.motion_planner.desired_x - self.cur_x) > 10:
                            v_x = -int(self.motion_planner.desired_x_vel)
                    #time.sleep(0.05)
                elif abs(self.motion_planner.desired_x - self.cur_x) < 5:
                    #self.tello.stop()
                    v_x = 0
                    self.in_position_x = True
                    #print(f'Greater than x {self.waypoints_x[self.waypoints_x_i - 1]}')
                    #print(f'{self.waypoints_x_i} {len(self.waypoints_x)}')
                    #print(f'x : {self.cur_x}')
                    #if len(self.waypoints_x) == self.waypoints_x_i:
                    if self.motion_planner.completed:
                        self.stopped_x = True
                        #time.sleep(1)
                        #self.tello.land()
                        #self.tello.end()

                #control for y-axis
                if abs(self.motion_planner.desired_y - self.cur_y) > 5:
                    self.in_position_y = False
                    self.stopped_y = False
                    if self.motion_planner.desired_y > self.cur_y:
                        v_y = 5
                        if self.motion_planner.desired_y_vel > v_y:
                            v_y = int(self.motion_planner.desired_y_vel)
                        if int(self.motion_planner.desired_y_vel) == 0:
                            v_y = 10
                        if abs(self.motion_planner.desired_y - self.cur_y) > 10:
                            v_y = int(self.motion_planner.desired_y_vel)
                    else:
                        v_y = -5
                        if self.motion_planner.desired_y_vel < v_y:
                            v_y = -int(self.motion_planner.desired_y_vel)
                        if int(self.motion_planner.desired_y_vel) == 0:
                            v_y = -10
                        if abs(self.motion_planner.desired_y - self.cur_y) > 10:
                            v_y = -int(self.motion_planner.desired_y_vel)
                    #time.sleep(0.05)
                elif abs(self.motion_planner.desired_y - self.cur_y) < 5:
                    #self.tello.stop()
                    v_y = 0
                    self.in_position_y = True
                    #print(f'Greater than y {self.waypoints_y[self.waypoints_y_i - 1]}')
                    #print(f'{self.waypoints_y_i} {len(self.waypoints_y)}')
                    #print(f'{self.cur_y}')
                    if self.motion_planner.completed:
                        self.stopped_y = True
                        #time.sleep(1)
                        #self.tello.land()
                        #self.tello.end()

                if self.stopped_x and self.stopped_y and self.motion_planner.completed:
                    self.stopped = True

                if time.perf_counter() - self.prev_timestamp > self.dt and self.in_position_x and self.in_position_y:
                    """if not self.waypoints_x_i -1 >= len(self.waypoints_x):
                        self.waypoints_x_i += 1
                    if not self.waypoints_y_i - 1 >= len(self.waypoints_y):
                        self.waypoints_y_i += 1"""
                    if not self.motion_planner.completed:
                        self.motion_planner.solve()
                    #print(time.perf_counter() - self.prev_timestamp)
                        self.prev_timestamp = time.perf_counter()
                        print(f'Solving {self.prev_timestamp}')

                self.tello.send_rc_control(v_x, v_y, 0, 0)

        print(f'Stopping X : {self.cur_x} Y : {self.cur_y}')

    def yaw_align(self):
        if not -1.0 < self.yaw < 1.0:
            self.tello.send_rc_control(0,0,0,20)
        else:
            self.tello.send_rc_control(0,0,0,0)
            #self.tello.stop()
            print(f'Alignment done yaw : {self.yaw}')
            time.sleep(5)
            self.yaw_aligned = True
            #self.tello.land()
            #self.tello.end()
        

    def init_seq(self):
        print("init seq")
        self.tello = Tello()
        self.tello.connect()
        self.tello.takeoff()
        self.control_thread = self.controller()
        self.is_flying = True

    def stop_callback(self,msg):
        self.stopped = True
        print("Stopping")
        self.tello.send_rc_control(0,0,0,0)
        self.tello.stop()
        self.tello.land()


    def pose_callback(self, msg):
        if self.is_flying == False and (not self.cur_x == None or not self.cur_y == None or not self.cur_z == None):
            self.init_seq()
            pass

        self.cur_x = float(msg.pose.position.x * 100)
        self.cur_y = float(msg.pose.position.y * 100)
        self.cur_z = float(msg.pose.position.z * 100)

        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        self.yaw = round(math.degrees(yaw),2)

        self.counter += 1

        if self.counter % 3:
            if not self.prev_x == None:
                self.v_x = (self.cur_x - self.prev_x) / (time.perf_counter() - self.prev_x_time)
                self.prev_x = self.cur_x
                self.prev_x_time = time.perf_counter()
                self.prev_speed_x = self.v_x
            else:
                self.prev_x_time = time.perf_counter()
                self.prev_x = self.cur_x

        if not self.yaw_aligned and self.is_flying:
            self.yaw_align()

        if self.motion_planner == None and self.yaw_aligned:
            self.motion_planner =  MotionPlanner([self.cur_x,self.cur_y,self.cur_z],[150,225,100], T=7, dt=0.5)
            self.motion_planner.solve()
            self.motion_planned = True
            self.prev_timestamp = time.perf_counter()
            self.dt = self.motion_planner.dt

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
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
class DroneShow(Node):
    def __init__(self):
        super().__init__('Drone_Show')

        self.destroyed = False

        self.kill = False

        self.start_localisation = False

        self.cur_x = None
        self.cur_y = None
        self.cur_z = None

        self.callbacks = {}
        self.subscribers = {}

        self.is_flying = False

        self.stopped = False
        self.stopped_x = False
        self.stopped_y = False
        self.stopped_z = False
        self.yaw_aligned = False

        self.in_position_x = False
        self.in_position_y = False
        self.in_position_z = False


        self.motion_planner = None
        self.motion_planned = False

        self.start = False

        self.counter = 0

        self.waypoints_x = [20, 20, 120, 120, 20, 150]
        self.waypoints_y = [100, 200, 200, 100, 100, 150]
        self.waypoints_x_i = 1
        self.waypoints_y_i = 1

        self.prev_timestamp = None
        self.dt = 0

        self.start_local = False

        for i in range(9):
            topic=f'/vrpn_client_node/drone{i}_cap/pose'
            new_cb = self.make_callback(topic, i)
            self.subscribers[i] = {}
            self.subscribers[i]['topic'] = topic
            self.subscribers[i]['name'] = None
            self.subscribers[i]['callback'] = new_cb
            self.subscribers[i]['scubscriber'] = self.create_subscription(PoseStamped, topic,self.subscribers[i]['callback'],1)

        self.control_thread = None

        self.show_controller = ControllerWindow(file_load_cb=self.file_load_cb, emergency_cb=self.emergency_cb, close_cb=self.close_cb, start_local_cb=self.start_local_cb, init_cb=self.init_cb)
        self.swarm = Swarm()

    def emergency_cb(self):
        self.kill = True
        print("Emergency callback")
        self.stop()

    def file_load_cb(self, anim_file_location):
        self.waypoints_x = [20]
        WaypointGenerator(anim_file_location)
        self.show_controller.update_data()
        print(Swarm.DRONE_DATA)
        
    def close_cb(self):
        while True:
            if self.show_controller.shutdown_completed:
                break
        self.destroyed = True
        self.destroy_node()
        rclpy.shutdown()

    def make_callback(self, topic, index):
        def callback(msg):
            if self.start_localisation:
                name = self.subscribers[index]['name']
                if not name == None:
                    try:
                        Swarm.DRONE_DATA[name]['x'] = float(msg.pose.position.x * 100)
                        Swarm.DRONE_DATA[name]['y'] = float(msg.pose.position.y * 100)
                        Swarm.DRONE_DATA[name]['z'] = float(msg.pose.position.z * 100)

                        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

                        self.yaw = round(math.degrees(yaw),2)

                        Swarm.DRONE_DATA[name]['yaw'] = yaw
                    except:
                        pass
        return callback

    def start_local_cb(self):
        for drone in Swarm.DRONE_DATA:
            drone = Swarm.DRONE_DATA[drone]
            topic = drone['topic']
            if not drone['topic'] == None:
                for index in self.subscribers:
                    if self.subscribers[index]['topic'] == topic:
                        self.subscribers[index]['name'] = drone['name']
        #print(self.subscribers)
        self.start_localisation = True

    @threaded
    def init_cb(self):
        self.swarm.initialise()
        self.swarm.connect()
        self.swarm.takeoff()
        time.sleep(2)
        self.swarm.land()
       
    def shutdown(self):
        self.show_controller.shutdown = True

    @threaded
    def controller(self, name):
        drone = Swarm.DRONE_DATA[name]
        while not self.kill:
            while not self.stopped:

                if self.yaw_aligned and self.motion_planned:
                    
                    des_x = self.motion_planner.desired_x
                    des_y = self.motion_planner.desired_y
                    des_z = self.motion_planner.desired_z

                    des_x_v = int(abs(self.motion_planner.desired_x_vel))
                    des_y_v = int(abs(self.motion_planner.desired_y_vel))
                    des_z_v = int(abs(self.motion_planner.desired_z_vel))

                    print(f'x : {round(self.cur_x,2)} des_x {des_x} y : {round(self.cur_y,2)} des_y {des_y} z : {round(self.cur_z,2)} des_z {des_z} yaw : {self.yaw}' , end='\r')
                    if not self.start:
                        self.tello.send_rc_control(0,0,0,0)
                    #Code for safety net            
                    if self.cur_x > 400 or self.cur_y > 350 or self.cur_z > 210 or self.cur_x < -40 or self.cur_y < 30:
                        print(f'Safety net activated {self.cur_x} {self.cur_y} {self.cur_z}')
                        self.stopped_x = True
                        self.stopped_y = True
                        self.stopped_z = True
                        self.tello.stop()
                        self.tello.send_rc_control(0,0,0,0)
                        time.sleep(1)
                        self.tello.land()
                        self.tello.end()

                    if self.prev_timestamp == None:
                        self.prev_timestamp = time.perf_counter()
                    
                    #Control for x-axis
                    if abs(des_x - self.cur_x) > 5:
                        self.in_position_x = False
                        self.stopped_x = False
                        if des_x > self.cur_x:
                            v_x = 10

                            if abs(des_x - self.cur_x) > 10 and des_x_v > 20:
                                v_x = int(min(des_x_v*0.9,20))

                            if abs(des_x - self.cur_x) > 20:
                                v_x = int(min(des_x_v,40))

                            if int(des_x_v) <= 20 and abs(des_x - self.cur_x) > 10:
                                v_x = 20
                        else:
                            v_x = -10

                            if abs(des_x - self.cur_x) > 10 and des_x_v > 20:
                                v_x = -int(min(des_x_v*0.9,20))

                            if abs(des_x - self.cur_x) > 20:
                                v_x = -int(min(des_x_v,40))

                            if int(des_x_v) <= 20 and abs(des_x - self.cur_x) > 10:
                                v_x = -20

                    elif abs(des_x - self.cur_x) < 5:
                        v_x = 0
                        self.in_position_x = True
                        if self.motion_planner.completed:
                            self.stopped_x = True

                    #Control for y-axis
                    if abs(des_y - self.cur_y) > 5:
                        self.in_position_y = False
                        self.stopped_y = False
                        if des_y > self.cur_y:
                            v_y = 10

                            if abs(des_y - self.cur_y) > 10 and des_y_v > 20:
                                v_y = int(min(des_y_v*0.9,20))

                            if abs(des_y - self.cur_y) > 20:
                                v_y = int(min(des_y_v,40))

                            if int(des_y_v) <= 20 and abs(des_y - self.cur_y) > 10:
                                v_y = 20
                        else:
                            v_y = -10

                            if abs(des_y - self.cur_y) > 10 and des_y_v > 20:
                                v_y = -int(min(des_y_v*0.9,20))

                            if abs(des_y - self.cur_y) > 20:
                                v_y = -int(min(des_y_v,40))

                            if int(des_y_v) <= 20 and abs(des_y - self.cur_y) > 10:
                                v_y = -20

                    elif abs(des_y - self.cur_y) < 5:
                        v_y = 0
                        self.in_position_y = True
                        if self.motion_planner.completed:
                            self.stopped_y = True

                    #Control for z-axis
                    if abs(des_z - self.cur_z) > 5:
                        self.in_position_z = False
                        self.stopped_z = False
                        if des_z > self.cur_z:
                            v_z = 10

                            if abs(des_z - self.cur_z) > 10 and des_z_v > 20:
                                v_z = int(min(des_z_v*0.9,20))

                            if abs(des_z - self.cur_z) > 20:
                                v_z = int(min(des_z_v,40))

                            if int(des_z_v) <= 20 and abs(des_z - self.cur_z) > 10:
                                v_z = 20
                        else:
                            v_z = -10

                            if abs(des_z - self.cur_z) > 10 and des_z_v > 20:
                                v_z = -int(min(des_z_v*0.9,20))

                            if abs(des_z - self.cur_z) > 20:
                                v_z = -int(min(des_z_v,40))

                            if int(des_z_v) <= 20 and abs(des_z - self.cur_z) > 10:
                                v_z = -20

                    elif abs(des_z - self.cur_z) < 5:
                        v_z = 0
                        self.in_position_z = True
                        if self.motion_planner.completed:
                            self.stopped_z = True

                    #Check if the wapoint has been achieved
                    if self.stopped_x and self.stopped_y and self.stopped_z and self.motion_planner.completed:
                        print("Motion Completed")
                        self.stopped = True

                    if time.perf_counter() - self.prev_timestamp > self.dt or self.in_position_x and self.in_position_y and self.in_position_z:
                        if not self.motion_planner.completed:
                            self.motion_planner.solve()
                            self.prev_timestamp = time.perf_counter()
                            print(f'Solving {self.prev_timestamp}')

                    self.tello.send_rc_control(v_x, v_y, v_z, 0)

            print(f'Stopping X : {self.cur_x} Y : {self.cur_y} Z : {self.cur_z}')
        #Send tello command to stop

    def yaw_align(self):
        if not -1.0 < self.yaw < 1.0:
            self.tello.send_rc_control(0,0,0,20)
        else:
            self.tello.send_rc_control(0,0,0,0)
            print(f'Alignment done yaw : {self.yaw}')
            time.sleep(5)
            self.yaw_aligned = True
        

    def init_seq(self):
        print("init seq")
        self.tello = Tello()
        self.tello.connect()
        print(self.tello.get_battery())
        self.tello.takeoff()
        self.control_thread = self.controller()
        self.is_flying = True

    def stop_callback(self,msg):
        self.stopped = True
        print("Stopping")
        self.tello.send_rc_control(0,0,0,0)
        self.tello.stop()
        self.tello.land()

    def start_callback(self, msg):
        print("Starting")
        self.start = True


    def pose_callback(self, msg):
        if self.is_flying == False and (not self.cur_x == None or not self.cur_y == None or not self.cur_z == None):
            self.init_seq()
            pass

        self.cur_x = float(msg.pose.position.x * 100)
        self.cur_y = float(msg.pose.position.y * 100)
        self.cur_z = float(msg.pose.position.z * 100)

        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

        self.yaw = round(math.degrees(yaw),2)

        if not self.yaw_aligned and self.is_flying:
            self.yaw_align()

        if self.motion_planner == None and self.yaw_aligned and self.start:
            self.motion_planner =  MotionPlanner([self.cur_x,self.cur_y,self.cur_z],[200,220,150], T=7, dt=1)
            self.motion_planner.solve()
            self.motion_planned = True
            self.prev_timestamp = time.perf_counter()
            self.dt = self.motion_planner.dt

def main(args=None):
    rclpy.init(args=args)

    drone_show = DroneShow()

    try:
        rclpy.spin(drone_show)
    except:
        while not drone_show.show_controller.shutdown_completed:
                pass
        drone_show.destroy_node()
    finally:
        if not drone_show.destroyed:
            print("Shutting down node gracefully")
            while not drone_show.show_controller.shutdown_completed:
                pass
            drone_show.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
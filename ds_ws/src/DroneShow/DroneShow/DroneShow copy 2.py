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
import copy

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

        self.callbacks = {}
        self.subscribers = {}

        self.is_flying = False

        self.cur_x=None
        self.cur_y=None
        self.cur_z=None

        self.stopped = False
        self.stopped_x = False
        self.stopped_y = False
        self.stopped_z = False
        self.yaw_aligned = False

        self.in_position_x = False
        self.in_position_y = False
        self.in_position_z = False

        self.start = False

        self.drone_nos = None
        self.waypoint_achieved = []
        self.waypoints_completed = False
        self.yaw_a_a = []
        self.yaw_a_c = False

        self.prev_timestamp = time.perf_counter()
        self.dt = 0

        self.DRONE_DATA = None

        self.start_local = False

        self.tellos = None

        for i in range(9):
            topic=f'/vrpn_client_node/drone{i}_cap/pose'
            new_cb = self.make_callback(topic, i)
            self.subscribers[i] = {}
            self.subscribers[i]['topic'] = topic
            self.subscribers[i]['name'] = None
            self.subscribers[i]['callback'] = new_cb
            self.subscribers[i]['scubscriber'] = self.create_subscription(PoseStamped, topic,self.subscribers[i]['callback'],1)

        self.control_thread = None

        self.show_controller = ControllerWindow(file_load_cb=self.file_load_cb, emergency_cb=self.emergency_cb, close_cb=self.close_cb, start_local_cb=self.start_local_cb, init_cb=self.init_cb, takeoff_cb=self.takeoff_cb, start_cb=self.start_cb, add_data_cb=self.add_data_cb, get_data=self.get_data)
        self.swarm = Swarm()

    def get_data(self):
        #print("asd")
        #print(self.swarm.DRONE_DATA)
        return self.swarm.DRONE_DATA

    def emergency_cb(self):
        self.kill = True
        self.swarm.swarm_controller.parallel(lambda i, tello: tello.send_rc_control(0,0,0,0))
        self.swarm.swarm_controller.parallel(lambda i, tello: tello.send_rc_control(0,0,0,0))
        self.swarm.swarm_controller.land()
        print("Emergency callback")
        #self.stop()

    def file_load_cb(self, anim_file_location):
        self.waypoint_achieved = []
        WG=WaypointGenerator(anim_file_location)
        self.swarm.DRONE_DATA=WG.get_data()
        self.show_controller.update_data(DRONE_DATA=self.swarm.DRONE_DATA)
        """self.DRONE_DATA = Swarm.DRONE_DATA"""
        for name in self.swarm.DRONE_DATA:
            self.waypoint_achieved.append(False)
        #print(self.waypoint_achieved)
        
    def close_cb(self):
        self.kill=True
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
                        self.swarm.DRONE_DATA[name]['x'] = float(msg.pose.position.x * 100)
                        self.swarm.DRONE_DATA[name]['y'] = float(msg.pose.position.y * 100)
                        self.swarm.DRONE_DATA[name]['z'] = float(msg.pose.position.z * 100)

                        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

                        yaw = round(math.degrees(yaw),2)

                        self.swarm.DRONE_DATA[name]['yaw'] = yaw
                        if time.perf_counter() - self.prev_timestamp > 0.1:
                            self.show_controller.update_data(DRONE_DATA=self.swarm.DRONE_DATA)
                            self.prev_timestamp = time.perf_counter()
                    except:
                        pass
        return callback

    @threaded
    def start_local_cb(self):
        for name in self.swarm.DRONE_DATA:
            drone = self.swarm.DRONE_DATA[name]
            topic = drone['topic']
            if not drone['topic'] == None:
                for index in self.subscribers:
                    if self.subscribers[index]['topic'] == topic:
                        self.subscribers[index]['name'] = drone['name']
        self.start_localisation = True

    @threaded
    def init_cb(self):
        self.yaw_a_a = []
        self.yaw_a_c = False
        self.swarm.initialise()
        for name in self.swarm.DRONE_DATA:
            self.yaw_a_a.append(False)
        self.drone_nos = len(self.yaw_a_a)
        #print(self.yaw_a_a)
        #print(self.drone_nos)

    @threaded
    def takeoff_cb(self):
        print("Takeoff")
        self.swarm.connect()
        self.tellos = self.swarm.swarm_controller.tellos
        self.swarm.takeoff()
        time.sleep(2)

    def start_cb(self):
        pass
        for name in copy.copy(self.swarm.DRONE_DATA):
            #self.swarm.DRONE_DATA['control_thread'] = self.controller(self.swarm.DRONE_DATA[name])
            self.controller(self.swarm.DRONE_DATA[name])

    def add_data_cb(self,name, topic, ip):
        self.swarm.DRONE_DATA[name]['topic'] = topic
        self.swarm.DRONE_DATA[name]['ip'] = ip
       
    def shutdown(self):
        if self.swarm.swarm_connected:
            try:
                self.swarm.swarm_controller.land()
            except:
                pass
        self.show_controller.shutdown = True
        self.kill = True
        if not self.destroyed:
            self.destroy_node()
            rclpy.shutdown()
            self.destroyed = True


    @threaded
    def controller(self, drone):
        print(drone['name'])
        tello=self.tellos[drone['id']]

        """while not self.kill:
            pass"""

        while not self.kill:
            if drone['mp'] == None:
                print("Planning Motion for ", drone['name'])
                start_pos = [drone['x'], drone['y'],drone['z']]
                des_pos = []
                drone['mp'] = 1

            
            while True:
                if self.swarm.swarm_connected:
                    #print("True")
                    
                    if not -5 < drone['yaw'] < 5 and not self.yaw_a_a[drone['id']]:
                        print(f'name: {drone["name"]},  {drone["yaw"]}', end="\r\r\r\r")
                        tello.send_rc_control(0,0,0,20)
                        #print("Aligning")
                    else:
                        tello.send_rc_control(0,0,0,0)
                        print("Alignment done")
                        self.yaw_a_a[drone['id']] = True
                        check_all_yaw = self.yaw_a_a.count(True)

                        if check_all_yaw >= self.drone_nos:
                            self.yaw_a_c = True
                            print("All Aligned")
                            break
            if self.yaw_a_c:
                break

        self.kill = True

        while not self.kill:
            if time.perf_counter() - self.prev_timestamp > 0.1:
                check_wp = self.waypoint_achieved.count(True)
                #print(check_wp)
                if self.stopped == False:
                        #print(drone['name'], end="\r\r\r")
                        if self.yaw_a_a[drone['id']]:
                            print("asd")
                            
                            

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
                else:
                    pass
                    print(f'Stopping X : {self.cur_x} Y : {self.cur_y} Z : {self.cur_z}')
        print("Kill")

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

def main(args=None):
    rclpy.init(args=args)

    drone_show = DroneShow()

    try:
        rclpy.spin(drone_show)
    except:
        """while not drone_show.show_controller.shutdown_completed:
                pass
        drone_show.destroy_node()"""
        pass
    finally:
        if not drone_show.destroyed:
            print("Shutting down node gracefully")
            drone_show.shutdown()
            if not drone_show.show_controller.shutdown_completed:
                pass
            drone_show.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
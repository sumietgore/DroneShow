"""
    Main controller for the ROS package. Either build the package using colcon or directly run this python file.

    The main components of this package are:
        Waypoint Generator (Reads the animation file and creates waypoints for the drone to follow)
        Path Planner (Quinitic polynomial based path planner. A path is planned every time between two waypoints.)
        Path Tracking (A custom location based controller to follow waypoints)
        Tello Controller (DJI Tello Py Controller. https://github.com/damiafuentes/DJITelloPy)      
"""

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
        print(self.waypoint_achieved)
        
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
                        if time.perf_counter() - self.prev_timestamp > 0.5:
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
        self.start = True

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
            motion_planner = None
            prev_timestamp = None
            waypoint_id = 0
            
            in_position_x = False
            in_position_y = False
            in_position_z = False

            stopped=False
            stopped_x = False
            stopped_y = False
            stopped_z = False

            v_x = 0
            v_y = 0
            v_z = 0

            if motion_planner == None:
                print("Planning Motion for ", drone['name'])
                start_pos = [drone['x'], drone['y'],drone['z']]
                des_pos = [drone['waypoint'][0][waypoint_id],drone['waypoint'][1][waypoint_id], drone['waypoint'][2][waypoint_id]]
                time1 = drone['time'][waypoint_id]
                if time1 == 0:
                    time1 = 5
                print(des_pos, time1)
                motion_planner = MotionPlanner(start_pos, des_pos, dt=0.5, T=time1)
                motion_planner.solve()
                waypoint_id += 1

            while True:
                if self.swarm.swarm_connected:
                    #print("True")
                    
                    if not -3 < drone['yaw'] < 3 and not self.yaw_a_a[drone['id']]:
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

        #self.kill = True

        prev_timestamp = time.perf_counter()

        while not self.kill:
                #print(check_wp)
            if not stopped:
                if not self.start:
                        tello.send_rc_control(0,0,0,0)

                self.waypoint_achieved[drone['id']] = False

                count_waypoiny_drones = self.waypoint_achieved.count(False)
                if count_waypoiny_drones >= self.drone_nos and self.waypoints_completed:
                    self.waypoints_completed = False
                
                #count_waypoiny_drones = self.waypoint_achieved.count(False)
                #if count_waypoiny_drones >= self.drone_nos:

                    #print(drone['name'], end="\r\r\r")
                if self.yaw_a_a[drone['id']] and not self.waypoints_completed:

                    des_x = motion_planner.desired_x
                    des_y = motion_planner.desired_y
                    des_z = motion_planner.desired_z

                    des_x_v = int(abs(motion_planner.desired_x_vel))
                    des_y_v = int(abs(motion_planner.desired_y_vel))
                    des_z_v = int(abs(motion_planner.desired_z_vel))

                    cur_x = drone['x']
                    cur_y = drone['y']
                    cur_z = drone['z']

                    #print(f'x : {round(self.cur_x,2)} des_x {des_x} y : {round(self.cur_y,2)} des_y {des_y} z : {round(self.cur_z,2)} des_z {des_z} yaw : {self.yaw}' , end='\r')
                    #Code for safety net            
                    if cur_x > 400 or cur_y > 350 or cur_z > 230 or cur_x < -40 or cur_y < 10:
                        print(f'Safety net activated {self.cur_x} {self.cur_y} {self.cur_z}')
                        stopped_x = True
                        stopped_y = True
                        stopped_z = True
                        tello.send_rc_control(0,0,0,0)
                        self.kill = True
                        #self.tello.stop()
                        #self.tello.send_rc_control(0,0,0,0)
                        #time.sleep(1)
                        #self.tello.land()
                        #self.tello.end()

                    if prev_timestamp == None:
                        prev_timestamp = time.perf_counter()
                    
                    #Control for x-axis
                    if abs(des_x - cur_x) > 7:
                        in_position_x = False
                        stopped_x = False
                        if des_x > cur_x:
                            v_x = 10

                            if abs(des_x - cur_x) > 10 and des_x_v > 20:
                                v_x = int(min(des_x_v*0.9,20))

                            if abs(des_x - cur_x) > 20:
                                v_x = int(min(des_x_v,40))

                            if int(des_x_v) <= 20 and abs(des_x - cur_x) > 10:
                                v_x = 15
                        else:
                            v_x = -10

                            if abs(des_x - cur_x) > 10 and des_x_v > 20:
                                v_x = -int(min(des_x_v*0.9,20))

                            if abs(des_x - cur_x) > 20:
                                v_x = -int(min(des_x_v,40))

                            if int(des_x_v) <= 20 and abs(des_x - cur_x) > 10:
                                v_x = -15

                    elif abs(des_x - cur_x) < 7:
                        v_x = 0
                        in_position_x = True
                        if motion_planner.completed:
                            stopped_x = True

                    #Control for y-axis
                    if abs(des_y - cur_y) > 7:
                        in_position_y = False
                        stopped_y = False
                        if des_y > cur_y:
                            v_y = 10

                            if abs(des_y - cur_y) > 10 and des_y_v > 20:
                                v_y = int(min(des_y_v*0.9,20))

                            if abs(des_y - cur_y) > 20:
                                v_y = int(min(des_y_v,40))

                            if int(des_y_v) <= 20 and abs(des_y - cur_y) > 10:
                                v_y = 15
                        else:
                            v_y = -10

                            if abs(des_y - cur_y) > 10 and des_y_v > 20:
                                v_y = -int(min(des_y_v*0.9,20))

                            if abs(des_y - cur_y) > 20:
                                v_y = -int(min(des_y_v,40))

                            if int(des_y_v) <= 20 and abs(des_y - cur_y) > 10:
                                v_y = -15

                    elif abs(des_y - cur_y) < 7:
                        v_y = 0
                        in_position_y = True
                        if motion_planner.completed:
                            stopped_y = True

                    #Control for z-axis
                    if abs(des_z - cur_z) > 7:
                        in_position_z = False
                        stopped_z = False
                        if des_z > cur_z:
                            v_z = 15

                            if abs(des_z - cur_z) > 10 and des_z_v > 20:
                                v_z = int(min(des_z_v*0.9,20))

                            if abs(des_z - cur_z) > 20:
                                v_z = int(min(des_z_v,40))

                            if int(des_z_v) <= 20 and abs(des_z - cur_z) > 10:
                                v_z = 25
                        else:
                            v_z = -15

                            if abs(des_z - cur_z) > 10 and des_z_v > 20:
                                v_z = -int(min(des_z_v*0.9,20))

                            if abs(des_z - cur_z) > 20:
                                v_z = -int(min(des_z_v,40))

                            if int(des_z_v) <= 20 and abs(des_z - cur_z) > 10:
                                v_z = -25

                    elif abs(des_z - cur_z) < 7:
                        v_z = 0
                        in_position_z = True
                        if motion_planner.completed:
                            stopped_z = True

                    #Check if the wapoint has been achieved
                    if stopped_x and stopped_y and stopped_z and motion_planner.completed:
                        print(f'{drone["name"]}Motion Completed')
                        stopped = True
                        self.waypoint_achieved[drone['id']] = True

                    if time.perf_counter() - prev_timestamp > motion_planner.dt or in_position_x and in_position_y and in_position_z:
                        if not motion_planner.completed:
                            motion_planner.solve()
                            prev_timestamp = time.perf_counter()
                            #print(f'Solving {prev_timestamp}')
                        

                    tello.send_rc_control(v_x, v_y, v_z, 0)
            else:
                count_waypoiny_drones = self.waypoint_achieved.count(True)
                tello.send_rc_control(0, 0, 0, 0)
                if count_waypoiny_drones >= self.drone_nos:
                    print(self.waypoint_achieved)
                    self.waypoints_completed = True
                if motion_planner.completed and self.start and self.waypoints_completed:
                    length = len(drone['waypoint'][0])
                    if waypoint_id <= length-1:
                        in_position_x = False
                        in_position_y = False
                        in_position_z = False

                        stopped=False
                        stopped_x = True
                        stopped_y = True
                        stopped_z = True

                        v_x = 0
                        v_y = 0
                        v_z = 0

                        start_pos = [drone['waypoint'][0][waypoint_id-1],drone['waypoint'][1][waypoint_id-1], drone['waypoint'][2][waypoint_id-1]]
                        des_pos = [drone['waypoint'][0][waypoint_id],drone['waypoint'][1][waypoint_id], drone['waypoint'][2][waypoint_id]]
                        time1 = drone['time'][waypoint_id]
                        if time1 == 0:
                            time1 = 5
                        print(des_pos, time)
                        motion_planner = MotionPlanner(start_pos, des_pos, dt=0.5, T=time1)
                        motion_planner.solve()
                        waypoint_id += 1
                        #print(f'Stopping X : {cur_x} Y : {cur_y} Z : {cur_z}')
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
        self.kill=True
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
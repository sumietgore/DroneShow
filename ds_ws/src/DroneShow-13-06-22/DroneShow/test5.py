import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from lib.tellopy import Tello
from lib.MotionPlanner import MotionPlanner
import time
from threading import Thread
import numpy as np

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class Test(Node):
    def __init__(self):
        super().__init__('Motion_Test')

        self.cur_x = None
        self.cur_y = None
        self.cur_z = None
        self.is_flying = False
        self.stopped = False

        self.wapoints_x = [25,75,150]
        self.wapoints_x_i = 1

        self.prev_timestamp = None
        self.dt = 1

        self.pose_subscriber = self.create_subscription(PoseStamped,'/vrpn_client_node/drone1_cap/pose',self.pose_callback,1)
        self.stop_subscriber = self.create_subscription(String, "/stop",self.stop_callback, 1)

        self.control_thread = Thread()
        self.control_thread.start()

    @threaded
    def controller(self):
        while not self.stopped:
            #print(f'x : {self.cur_x} y : {self.cur_y} z : {self.cur_z}')
            
            if self.cur_x > 175 or self.cur_y > 150 or self.cur_z > 100:
                print(f'Safety net activated {self.cur_x} {self.cur_y} {self.cur_z}')
                self.stopped = True
                self.tello.stop()
                self.tello.send_rc_control(0,0,0,0)
                time.sleep(1)
                self.tello.land()
                self.tello.end()

            if self.prev_timestamp == None:
                self.prev_timestamp = time.perf_counter()
            if abs(self.wapoints_x[self.wapoints_x_i - 1] - self.cur_x) > 5:
                if self.wapoints_x[self.wapoints_x_i - 1] > self.cur_x:
                    self.tello.send_rc_control(0,20,0,0)
                else:
                    self.tello.send_rc_control(0,-20,0,0)
                #time.sleep(0.05)
            elif abs(self.wapoints_x[self.wapoints_x_i - 1] - self.cur_x) < 5:
                self.tello.stop()
                self.tello.send_rc_control(0,0,0,0)
                print(f'Greater than {self.wapoints_x[self.wapoints_x_i - 1]}')
                print(f'{self.wapoints_x_i} {len(self.wapoints_x)}')
                print(f'{self.cur_x}')
                if len(self.wapoints_x) == self.wapoints_x_i:
                    self.stopped = True
                    time.sleep(1)
                    self.tello.land()
                    self.tello.end()
                if time.perf_counter() - self.prev_timestamp > self.dt:
                    self.wapoints_x_i += 1

    def init_seq(self):
        print("init seq")
        self.tello = Tello()
        self.tello.connect()
        self.tello.takeoff()
        self.controller()
        self.is_flying = True

    def stop_callback(self,msg):
        self.tello.stop()

    def pose_callback(self, msg):
        if self.is_flying == False and (not self.cur_x == None or not self.cur_y == None or not self.cur_z == None):
            self.init_seq()
            #pass

        self.cur_x = float(msg.pose.position.x * 100)
        self.cur_y = float(msg.pose.position.y * 100)
        self.cur_z = float(msg.pose.position.z * 100)

        #if not self.stopped:
        #    print(f'x : {self.cur_x} y : {self.cur_y} z : {self.cur_z} roll : {roll} pitch : {pitch} yaw : {yaw}')

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
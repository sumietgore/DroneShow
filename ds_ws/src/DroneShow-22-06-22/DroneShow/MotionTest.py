import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from djitellopy import Tello
from lib.MotionPlanner import MotionPlanner
import time
from threading import Thread
import numpy as np

class Test(Node):
    def __init__(self):
        super().__init__('Motion_Test')

        self.cur_x = None
        self.cur_y = None
        self.cur_z = None

        self.is_flying = False

        self.des_pos = [100,100,100]

        self.motion_planner = None

        self.T = 3
        self.dt = 0.2

        self.last_cmd = None
        self.motion_planned = False

        self.pose_subscriber = self.create_subscription(PoseStamped,'/vrpn_client_node/tello_5FC349/pose',self.pose_callback,1)

        self.tello = Tello()
        self.tello.connect()

        self.landed = False
        self.motion_counter = 0

    def pose_callback(self, msg):
        if not self.cur_x == None and not self.cur_y == None and not self.cur_z == None and not self.is_flying:
            if not self.landed:
                self.takeoff()

        self.cur_x = float(msg.pose.position.x * 100)
        self.cur_y = float(msg.pose.position.y * 100)
        self.cur_z = float(msg.pose.position.z * 100)

        if self.cur_x > 120.0 or self.cur_y > 120.0 or self.cur_z > 120.0:
            self.motion_planner.completed = True
            self.tello.send_rc_control(0,0,0,0)
            self.tello.stop()
            time.sleep(1)
            self.tello.land()
            self.is_flying = False

        #print(msg.header.stamp.nanosec)

        print(self.cur_x, self.cur_y, self.cur_z)

        if self.is_flying:
            if not self.motion_planner.completed:
                if self.motion_counter % 5 == 1 and self.motion_planned:
                    if abs(self.cur_x - self.motion_planner.desired_x) > 15.0 or abs(self.cur_x - self.motion_planner.desired_x) > 15.0 or abs(self.cur_x - self.motion_planner.desired_x) > 15.0:
                            self.motion_planner.recalculate([self.cur_x, self.cur_y, self.cur_z])
                #print('asd')
                if self.last_cmd == None:
                    #self.last_cmd = msg.header.stamp.nanosec
                    self.last_cmd = time.perf_counter()
                    print(f'Start time :{self.last_cmd}')
                    self.motion_planner.solve()
                    self.tello.send_rc_control(min(int(self.motion_planner.desired_x_vel),50), min(int(self.motion_planner.desired_y_vel),50), min(int(self.motion_planner.desired_z_vel),50), 0)
                #print(time.perf_counter() - self.last_cmd)
                if time.perf_counter() - self.last_cmd >= self.dt:
                    self.last_cmd = time.perf_counter()
                    self.motion_planner.solve()
                    print(self.motion_planner.desired_x_vel, self.motion_planner.desired_y_vel, self.motion_planner.t)
                    self.tello.send_rc_control(min(int(self.motion_planner.desired_x_vel),50), min(int(self.motion_planner.desired_y_vel),50), min(int(self.motion_planner.desired_z_vel),50), 0)
                    self.motion_counter += 1
            else:
                print(f'End time :{self.last_cmd}')
                self.tello.send_rc_control(0,0,0,0)
                print(f'last_position {self.cur_x}, {self.cur_y}, {self.cur_z}')
                self.tello.land()
                self.is_flying = False

    def takeoff(self):
        print('Takeoff')
        self.motion_planner = MotionPlanner([self.cur_x, self.cur_y, self.cur_z], self.des_pos, T=self.T, dt = self.dt)
        print(self.motion_planner.completed)
        self.motion_planned = True
        self.tello.takeoff()
        self.is_flying = True
    

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

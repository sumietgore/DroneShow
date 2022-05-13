import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from djitellopy import Tello
from lib.MotionPlanner import MotionPlanner
import time

class Test(Node):
    def __init__(self):
        super().__init__('Motion_Test')

        self.cur_x = None
        self.cur_y = None
        self.cur_z = None

        self.is_flying = False

        self.des_pos = [100,100,100]

        self.motion_planner = None

        self.T = 4
        self.dt = 0.1

        self.last_cmd = None
        self.motion_planned = False

        self.pose_subscriber = self.create_subscription(PoseStamped,'/vrpn_client_node/tello_5FC349/pose',self.pose_callback,10)

        self.tello = Tello()
        self.tello.connect()

    def pose_callback(self, msg):
        print('recievd msg')
        if not self.cur_x == None and not self.cur_y == None and not self.cur_z == None:
            self.takeoff()

        self.cur_x = float(msg.pose.position.x * 100)
        self.cur_y = float(msg.pose.position.y * 100)
        self.cur_z = float(msg.pose.position.z * 100)


        if self.is_flying:
            if not self.motion_planner.completed:
                if self.last_cmd == None:
                    self.last_cmd = time.perf_counter()
                if time.perf_counter() - self.last_cmd <= self.dt:
                    self.motion_planner.solve()
                    print(self.motion_planner.des_x_vel)
                    self.tello.send_rc_control(self.motion_planner.des_x_vel, self.motion_planner.des_y_vel, self.motion_planner.des_z_vel, 0)
                    if abs(self.cur_x - self.motion_planner.desired_x) > 10 or abs(self.cur_x - self.motion_planner.desired_x) > 10 or abs(self.cur_x - self.motion_planner.desired_x) > 10:
                        self.motion_planner.recalculate([self.cur_x, self.cur_y, self.cur_z])
            else:
                self.tello.land()
                self.is_flying = False

        

    def takeoff(self):
        print('Takeoff')
        self.tello.takeoff()
        self.motion_planner = MotionPlanner([self.cur_x, self.cur_y, self.cur_z], self.des_pos, T=4, dt = self.dt)
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

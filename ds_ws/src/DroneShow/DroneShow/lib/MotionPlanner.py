import numpy as np
from math import cos, sin
import time
from djitellopy import Tello

class MotionPlanner():
    def __init__(self, start_pos, des_pos, T=0.0, dt = 0.1, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.x_c = None
        self.y_c = None
        self.z_c = None

        self.completed = False

        self.dt = round(dt,3)
        self.T = round(T,3)
        self.t = round(0.0,3)

        #Solve the coefficients A, b_x, b_y and b_z for quintic polynomial
        self.__solve__coeff()

    def __solve__coeff(self):
        """"
        Calculate A, b_x, b_y and b_z
        """
        A = np.array(
            [[0, 0, 0, 0, 0, 1],
             [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
             [0, 0, 0, 0, 1, 0],
             [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
             [0, 0, 0, 2, 0, 0],
             [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
            ])

        b_x = np.array(
            [[self.start_x],
             [self.des_x],
             [self.start_x_vel],
             [self.des_x_vel],
             [self.start_x_acc],
             [self.des_x_acc]
            ])

        b_y = np.array(
            [[self.start_y],
             [self.des_y],
             [self.start_y_vel],
             [self.des_y_vel],
             [self.start_y_acc],
             [self.des_y_acc]
            ])

        b_z = np.array(
            [[self.start_z],
             [self.des_z],
             [self.start_z_vel],
             [self.des_z_vel],
             [self.start_z_acc],
             [self.des_z_acc]
            ])

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)

    def __calculate_position(self, c, t):
        return round(float(c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]),1)
    
    def solve(self):
        if self.t <= self.T:
            des_x_pos = self.__calculate_position(self.x_c, self.t)
            des_y_pos = self.__calculate_position(self.y_c, self.t)
            des_z_pos = self.__calculate_position(self.z_c, self.t)
            self.t = round(self.t + self.dt,3)
            print(f'x:{des_x_pos} ,   y:{des_y_pos} ,     z:{des_z_pos}     {self.t}')
            
            if self.T == self.t:
                self.completed = True
        else:
            print("Trajectory completed")
            self.completed = True


if __name__ == '__main__':
    T = 3.0

    waypoints = [[0,0,0],[200,100,0],[0,100,100],[0,0,0]]

    MP = MotionPlanner(waypoints[0], waypoints[1], T, dt=0.10)
    
    for i in range(len(waypoints) - 1):
        MP = MotionPlanner(waypoints[i], waypoints[i+1], T, dt=0.10)
        print(waypoints[i])
        while not MP.completed:
            MP.solve()
            time.sleep(MP.dt)
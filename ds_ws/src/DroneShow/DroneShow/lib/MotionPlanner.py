import numpy as np
import time

class MotionPlanner():
    """
    Creates a quintic polynomial motion planner for the drone
    """
    def __init__(self, start_pos = [0,0,0], des_pos = [0,0,0], T=0.0, dt = 0.1, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.set(start_pos, des_pos, T=T, dt = dt, start_vel=start_vel, des_vel=des_vel, start_acc=start_acc, des_acc=des_acc)
        
    def set(self, start_pos, des_pos, T=0.0, dt = 0.1, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):

        self.f_start_x = start_pos[0]
        self.f_start_y = start_pos[1]
        self.f_start_z = start_pos[2]

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

        self.desired_x = None
        self.desired_y = None
        self.desired_z = None

        self.desired_x_vel = None
        self.desired_y_vel = None  
        self.desired_z_vel = None

        self.desired_x_acc = None
        self.desired_y_acc = None  
        self.desired_z_acc = None

        self.completed = False

        self.dt = round(dt,3)
        self.T = round(T,3)
        self.t = round(0.0,3)
        self.t_t = round(0.0,3)

        #Solve the coefficients A, b_x, b_y and b_z for quintic polynomial
        self.__solve__coeff()

    def recalculate(self, current_pos):
        """
        Recalculate the coefficients if the drone is put of position
        """

        self.start_x_vel = (current_pos[0] - self.f_start_x)/self.t_t
        self.start_y_vel = (current_pos[1] - self.f_start_y)/self.t_t
        self.start_z_vel = (current_pos[2] - self.f_start_z)/self.t_t

        self.start_x = current_pos[0]
        self.start_y = current_pos[1]
        self.start_z = current_pos[2]

        self.start_x_acc = self.start_x_vel / self.t_t
        self.start_y_acc = self.start_y_vel / self.t_t
        self.start_z_acc = self.start_z_vel / self.t_t

        self.T = self.T - self.t
        self.t = 0

        self.__solve__coeff(recal=True)

    def __solve__coeff(self, recal = False):
        """"
        Calculate A, b_x, b_y and b_zdi
        """

        if recal == True:
            print("Recalculating")

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
    
    def __calculate_velocity(self, c, t):
        return round(float(5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]),1)

    def __calculate_acceleration(self, c, t):
        return round(float(20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]),1)
    
    def solve(self):
        """
        Solve the motion planner for particular timeframe
        """
        if self.t <= self.T:

            #Desired position calcultation
            self.desired_x = self.__calculate_position(self.x_c, self.t)
            self.desired_y = self.__calculate_position(self.y_c, self.t)
            self.desired_z = self.__calculate_position(self.z_c, self.t)

            #Desired velocity calculation
            self.desired_x_vel = self.__calculate_velocity(self.x_c, self.t)
            self.desired_y_vel = self.__calculate_velocity(self.y_c, self.t)
            self.desired_z_vel = self.__calculate_velocity(self.z_c, self.t)

            #Desired acc calculation
            self.desired_x_acc = self.__calculate_acceleration(self.x_c, self.t)
            self.desired_y_acc = self.__calculate_acceleration(self.y_c, self.t)
            self.desired_z_acc = self.__calculate_acceleration(self.z_c, self.t)

            self.t = round(self.t + self.dt,3)
            self.t_t = round(self.t + self.dt,3)
            #print(f'x:{self.desired_x} ,   y:{self.desired_y} ,     z:{self.desired_z}     {self.t}')
            
            if self.T == self.t:
                self.completed = True
        else:
            self.completed = True

        if self.completed:
            print("Completed")


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
from lib import QuinticTrajectoryPlanner

if __name__ == '__main__':
    T = 5

    drones = {}

    waypoints = [[0,0,0],[100,0,0],[100,100,0],[0,100,0],[0,0,0]]

    for i in range(1):
        traj = QuinticTrajectoryPlanner(waypoints[i], waypoints[(i + 1)], T, logger=True)
        traj.solve()
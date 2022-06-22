import csv
from .Swarm import Swarm

class WaypointGenerator():
    def __init__(self, file_location=None):
        self.file = file_location
        self.id = 0

        self.drone_data = {}

        if not self.file == None or not self.file == "":

            with open(self.file, newline='') as csvfile:
                spamreader = csv.reader(csvfile, delimiter=',')
                for row in spamreader:
                    if not row[0] in self.drone_data:
                        self.drone_data[row[0]] = {}
                        self.drone_data[row[0]]['name'] = row[0]
                        self.drone_data[row[0]]['id'] = self.id
                        self.drone_data[row[0]]['waypoint'] = [[], [], []]
                        self.drone_data[row[0]]['time'] = []
                        self.drone_data[row[0]]['ip'] = None
                        self.drone_data[row[0]]['topic'] = None
                        self.drone_data[row[0]]['subscriber'] = None
                        self.drone_data[row[0]]['callback'] = None
                        self.drone_data[row[0]]['x'] = 0
                        self.drone_data[row[0]]['y'] = 0
                        self.drone_data[row[0]]['z'] = 0
                        self.drone_data[row[0]]['yaw'] = 0
                        self.drone_data[row[0]]['v_data'] = 0
                        self.drone_data[row[0]]['mp'] = None
                        self.drone_data[row[0]]['state'] = None
                        self.drone_data[row[0]]['control_thread'] = None
                        self.id += 1

                    if row[2] == '2':
                        self.drone_data[row[0]]['waypoint'][2].append(float(row[4])*100)
                    elif row[2] == '1':
                        self.drone_data[row[0]]['waypoint'][1].append(float(row[4])*100)
                    elif row[2] == '0':
                        self.drone_data[row[0]]['waypoint'][0].append(float(row[4])*100)

                    time = float(row[3])/15

                    if not time in self.drone_data[row[0]]['time']:
                        self.drone_data[row[0]]['time'].append(time)
            
            for key in self.drone_data:
                if not len(self.drone_data[key]['waypoint'][0]) == len(self.drone_data[key]['time']) or not len(self.drone_data[key]['waypoint'][1]) == len(self.drone_data[key]['time']) or not len(self.drone_data[key]['waypoint'][2]) == len(self.drone_data[key]['time']):
                    self.drone_data = {}
                    break

            Swarm.DRONE_DATA = self.drone_data

    def get_data(self):
        return self.drone_data

if __name__ == "__main__":
    wapoint = WaypointGenerator()
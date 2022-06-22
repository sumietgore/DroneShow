import csv

class WaypointGenerator():
    def __init__(self):
        self.file = ""

        self.drone_data = {}

        with open('test.dsaf', newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',')
            for row in spamreader:
                if not row[0] in self.drone_data:
                    self.drone_data[row[0]] = {}
                    self.drone_data[row[0]]['waypoint'] = [[], [], []]
                    self.drone_data[row[0]]['time'] = []

                #print(float(row[4]))

                if row[2] == '2':
                    self.drone_data[row[0]]['waypoint'][2].append(float(row[4])*100)
                elif row[2] == '1':
                    self.drone_data[row[0]]['waypoint'][1].append(float(row[4])*100)
                elif row[2] == '0':
                    self.drone_data[row[0]]['waypoint'][0].append(float(row[4])*100)

                time = float(row[3])/5

                if not time in self.drone_data[row[0]]['time']:
                    self.drone_data[row[0]]['time'].append(time)
        
        for key in self.drone_data:
            if not len(self.drone_data[key]['waypoint'][0]) == len(self.drone_data[key]['time']) or not len(self.drone_data[key]['waypoint'][1]) == len(self.drone_data[key]['time']) or not len(self.drone_data[key]['waypoint'][2]) == len(self.drone_data[key]['time']):
                self.drone_data = {}
                break

        print(self.drone_data)

if __name__ == "__main__":
    wapoint = WaypointGenerator()
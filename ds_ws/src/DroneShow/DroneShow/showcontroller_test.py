from re import X
import PySimpleGUI as sg
from threading import Thread
import time

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class ControllerWindow():

    DRONE_DATA=[["Drone 1",1,2,3,4],["Drone 2",1,2,3,4]]

    TOPAREA = [sg.Button('Init', key='-INIT-',disabled = False),sg.In(key="-AnimFile-",visible=False),sg.FileBrowse("Select Animation File", file_types=(("Drone Show Animation File", "*.dsaf"),)),sg.Button("Configure Drone", key="-DRONE-CONFIG-", disabled=True),sg.Button("Compute Trajectory", disabled=True), sg.Button("Start Drone Show", disabled=True), sg.Button("Emergency Stop", disabled=True)]
    DRONEINFOAREA = [sg.Table(values=DRONE_DATA, headings=["Drone Name","X Value","Y Value", "Z Value", "Yaw Value"],key='-TABLE-',
                enable_events=True,
                enable_click_events=True,size=(100,50) )]

    LAYOUT = [TOPAREA, DRONEINFOAREA]
    WINDOW = sg.Window("DroneShow Controller",layout=LAYOUT)

    def __init__(self):
        self.window = ControllerWindow.WINDOW

        self.drone_data = ControllerWindow.DRONE_DATA

        self.selected_index = None

        self.current_state = "None"

        self.initialised = False
        self.last_update_timestamp = time.perf_counter()

        self.window_thread = self.MainWindow()

    def update(self,x,y,z, yaw):
        #print('updating')
        if time.perf_counter() - self.last_update_timestamp > 0.2:
            self.last_update_timestamp = time.perf_counter()
            self.drone_data[0][1] = x
            self.drone_data[0][2] = y
            self.drone_data[0][3] = z
            self.drone_data[0][4] = yaw
        
            self.window['-TABLE-'].update(values=self.drone_data)

    def update1(self):
        self.window['-TABLE-'].update(values=self.drone_data)

    @staticmethod
    def DroneConfigWindow(data, index):
        drone_config_layout = [[sg.Text("Drone Name", size=(25,1)),sg.Input(default_text=data[index][0])],
        [sg.Text("Localisation Topic Name", size=(25,1)), sg.In(default_text='Please enter ROS localisation topic')],
        [sg.Text("Drone IP Address", size=(25,1)), sg.In(default_text='Please enter IP address of the drone')]]
        drone_config_window = sg.Window("Drone Config", layout= drone_config_layout)
        while True:
            event, values = drone_config_window.read()
            if event == sg.WIN_CLOSED:
                break

    @threaded
    def MainWindow(self):
        while True:
            event, values = self.window.read()
            self.initialised = True
            print(event)
            if event == sg.WIN_CLOSED:
                break
            if event == 'Cancel': # if user closes window or clicks cancel
                answer = sg.PopupYesNo("Are you sure you want to exit the application? Some functions might be running in background.")
                if answer == "Yes":
                    break
                elif answer == "No" or "None":
                    pass
                else:
                    break
            elif event == "-TABLE-":
                print(event, values[event])
                self.selected_index = values[event][0]
            elif event =="-DRONE-CONFIG-":
                if not self.selected_index == None:   
                    ControllerWindow.DroneConfigWindow(self.drone_data, self.selected_index)
                    print("Closing Drone Config Window")
                else:
                    sg.Popup("Please select a drone from the table.")
            elif event == '-UPDATE-':
                self.update1()
            elif event == '-INIT-':
                self.initialised = True

        self.window.close()


if __name__ == "__main__":
    window = ControllerWindow()
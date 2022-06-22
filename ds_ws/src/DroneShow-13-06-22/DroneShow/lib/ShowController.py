import PySimpleGUI as sg
from threading import Thread


class ControllerWindow():

    DRONE_DATA=[["Drone 1",1,2,3,4],["Drone 2",1,2,3,4]]

    TOPAREA = [sg.In(key="-AnimFile-",visible=False),sg.FileBrowse("Select Animation File", file_types=(("Drone Show Animation File", "*.dsaf"),)),sg.Button("Configure Drone", key="-DRONE-CONFIG-", disabled=False),sg.Button("Compute Trajectory", disabled=True), sg.Button("Start Drone Show", disabled=True), sg.Button("Emergency Stop", disabled=True)]
    DRONEINFOAREA = [sg.Table(values=DRONE_DATA, headings=["Drone Name","X Value","Y Value", "Z Value", "Yaw Value"],key='-TABLE-',
                enable_events=True,
                enable_click_events=True, )]

    LAYOUT = [TOPAREA, DRONEINFOAREA]
    WINDOW = sg.Window("DroneShow Controller",layout=LAYOUT)

    def __init__(self):
        self.window = ControllerWindow.WINDOW

        self.selected_index = None

        self.current_state = "None"

        self.window_thread = Thread(target=ControllerWindow.MainWindow, args=(self,))
        self.window_thread.start()
        while self.window_thread.is_alive():
            pass

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

    @staticmethod
    def MainWindow(self):
        while True:
            event, values = self.window.read()
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
                    ControllerWindow.DroneConfigWindow(ControllerWindow.DRONE_DATA, self.selected_index)
                    print("Closing Drone Config Window")
                else:
                    sg.Popup("Please select a drone from the table.")

        self.window.close()


if __name__ == "__main__":
    window = ControllerWindow()
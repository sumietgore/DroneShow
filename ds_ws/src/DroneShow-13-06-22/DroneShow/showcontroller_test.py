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
    def __init__(self):
        #Varibales
        self.anim_file_loaded = False
        self.anim_file_location = None

        self.initialised = False
        self.show_started = False

        self.selected_index = None

        self.last_update_timestamp = time.perf_counter()


        self.drone_data=[["Drone 1",1,2,3,4,100,'locale','ip'],["Drone 2",1,2,3,4,100,'locale','ip']]
        self.drone_data = []

        self.top_area = [sg.Button('Browse',key='-BROWSE-'), sg.Button("Configure Drone", key="-DRONECONFIG-", disabled=True), 
            sg.Button('Init', key='-INIT-',disabled = True), sg.Button("Start Drone Show",key='-START-' ,disabled=True), 
            sg.Button("Emergency Stop", disabled=True)]
        
        self.drone_info_area = [sg.Table(values=self.drone_data, headings=["Drone Name","X","Y", "Z", "Yaw", "Battery"],key='-TABLE-',
                    enable_events=True,
                    enable_click_events=True,col_widths=[100,50,50,50,50,50],num_rows=20, auto_size_columns=True, expand_x=True )]

        self.win_layout = [self.top_area, self.drone_info_area]
        self.window = sg.Window("DroneShow Controller",layout=self.win_layout)

        self.window_thread = self.MainWindow()

    def update(self,drone_id,x,y,z, yaw):
        if time.perf_counter() - self.last_update_timestamp > 0.3:
            self.last_update_timestamp = time.perf_counter()
            self.drone_data[drone_id][1] = x
            self.drone_data[drone_id][2] = y
            self.drone_data[drone_id][3] = z
            self.drone_data[drone_id][4] = yaw
        
            self.window['-TABLE-'].update(values=self.drone_data)

    def DroneConfigWindow(self):
        index = self.selected_index
        drone_config_layout = [[sg.Text("Drone Name", size=(25,1)),sg.Input(default_text=self.drone_data[index][0])],
        [sg.Text("Localisation Topic Name",size=(25,1)), sg.Input(default_text=self.drone_data[index][6], key='-LOCALE-')],
        [sg.Text("Drone IP Address", size=(25,1)), sg.Input(default_text=self.drone_data[index][7],key='-IP-')],
        [sg.Button('Ok', key='-OK-'), sg.Button('Cancel', key='-CANCEL-')]]
        def window():
            drone_config_window = sg.Window("Drone Config", layout= drone_config_layout)
            while True:
                event, values = drone_config_window.read()
                if event == sg.WIN_CLOSED:
                    break
                elif event == '-OK-':
                    self.drone_data[index][6] = values['-LOCALE-']
                    self.drone_data[index][7] = values['-IP-']
                    break
                elif event == '-CANCEL-':
                    break
            drone_config_window.close()
        window()


    @threaded
    def MainWindow(self):
        while True:
            event, values = self.window.read()
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
                self.selected_index = values[event][0]
            elif event =="-DRONECONFIG-":
                if not self.selected_index == None: 
                    print(self.selected_index)  
                    self.DroneConfigWindow()
                    #print("Closing Drone Config Window")
                else:
                    sg.Popup("Please select a drone from the table.")
            elif event == '-BROWSE-':
                file = sg.popup_get_file('',file_types=(("Drone Show Animation File", "*.dsaf"),) ,multiple_files=False, no_window=True)
                if file == ():
                    print('No Files Selected')
                else:
                    
                    self.anim_file_location = file
                    self.anim_file_loaded = True
                    self.window['-DRONECONFIG-'].update(disabled=False)
                    self.window['-INIT-'].update(disabled=False)
                    print(file)
            elif event == '-UPDATE-':
                self.update1()
            elif event == '-INIT-':
                self.initialised = True
                self.window['-START-'].update(disabled=False)
                self.window['-DRONECONFIG-'].update(disabled=True)

        self.window.close()

if __name__ == "__main__":
    window = ControllerWindow()
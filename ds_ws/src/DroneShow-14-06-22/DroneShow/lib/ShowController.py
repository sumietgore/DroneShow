import PySimpleGUI as sg
from threading import Thread
from .Swarm import Swarm
import time

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class ControllerWindow():
    def __init__(self, file_load_cb=None, emergency_cb=None, close_cb=None,start_local_cb=None, init_cb=None):
        #Varibales
        self.anim_file_loaded = False
        self.anim_file_location = None

        self.initialised = False
        self.show_started = False

        self.selected_index = None

        self.start_localisation=False

        self.last_update_timestamp = time.perf_counter()

        self.file_load_cb = file_load_cb
        self.emergency_cb = emergency_cb
        self.close_cb=close_cb
        self.start_local_cb=start_local_cb
        self.init_cb=init_cb

        self.prev_update_time = time.perf_counter()

        self.drone_data = []

        self.shutdown = False
        self.shutdown_completed = False

        self.data_thread=None

        self.top_area = [sg.Button('Browse',key='-BROWSE-'), sg.Button("Configure Drone", key="-DRONECONFIG-", disabled=True),sg.Button('Start Localisation',key='-STARTLOCAL-', disabled=True), 
            sg.Button('Init', key='-INIT-',disabled = True), sg.Button("Start Drone Show",key='-START-' ,disabled=True), 
            sg.Button("Emergency Stop", disabled=True), sg.Button('Close', key='-CANCEL-')]
        
        self.drone_info_area = [sg.Table(values=self.drone_data, headings=["Drone Name","X","Y", "Z", "Yaw", "Battery"],key='-TABLE-',
                    enable_events=True,
                    enable_click_events=True,col_widths=[100,50,50,50,50,50],num_rows=20, auto_size_columns=True, expand_x=True, expand_y=True )]

        self.win_layout = [self.top_area, self.drone_info_area]
        self.window = sg.Window("DroneShow Controller",layout=self.win_layout, disable_close=True, size=(1200,500))

        self.window_thread = self.MainWindow()
        #self.data_thread = self.update_data_thread()

    def update_data(self):
        drone_data = []
        for index in Swarm.DRONE_DATA:
            drone = Swarm.DRONE_DATA[index]
            name = drone["name"]
            drone_data.append([name, drone['x'], drone['y'], drone['z'], drone['yaw'], 0])
        self.drone_data = drone_data
        self.window['-TABLE-'].update(values=self.drone_data)

    @threaded
    def update_data_thread(self):
        if self.prev_update_time == None:
            self.prev_update_time = time.perf_counter()
        while not self.shutdown:
            if self.shutdown:
                break
            if self.start_localisation:
                if time.perf_counter() - self.prev_update_time > 0.2:
                    self.update_data()
                    self.prev_update_time=time.perf_counter()

    def DroneConfigWindow(self):
        index = self.selected_index
        name = self.drone_data[index][0]
        drone_config_layout = [[sg.Text("Drone Name", size=(25,1)),sg.Input(default_text=self.drone_data[index][0], disabled=True)],
        [sg.Text("Localisation Topic Name",size=(25,1)), sg.Input(default_text=Swarm.DRONE_DATA[name]['topic'], key='-LOCALE-')],
        [sg.Text("Drone IP Address", size=(25,1)), sg.Input(default_text=Swarm.DRONE_DATA[name]['ip'],key='-IP-')],
        [sg.Button('Ok', key='-OK-'), sg.Button('Cancel', key='-CANCEL-')]]
        def window():
            drone_config_window = sg.Window("Drone Config", layout= drone_config_layout)
            while True:
                if self.shutdown:
                    break
                event, values = drone_config_window.read(timeout=100)
                if event == sg.WIN_CLOSED:
                    break
                elif event == '-OK-':
                    Swarm.DRONE_DATA[name]['topic'] = values['-LOCALE-']
                    Swarm.DRONE_DATA[name]['ip'] = values['-IP-']
                    break
                elif event == '-CANCEL-':
                    break
            drone_config_window.close()
        window()

    @threaded
    def MainWindow(self):
        while True:
            if self.shutdown:
                time.sleep(4)
                break
            if self.start_localisation:
                if time.perf_counter() - self.prev_update_time > 0.2:
                    self.update_data()
                    self.prev_update_time=time.perf_counter()
            event, values = self.window.read(timeout=50)
            if event == sg.WIN_CLOSED:
                break
            if event == '-CANCEL-': # if user closes window or clicks cancel
                answer = sg.PopupYesNo("Are you sure you want to exit the application? Some functions might be running in background.")
                if answer == "Yes":
                    break
                elif answer == "No" or "None":
                    pass
                else:
                    break
            elif event == "-TABLE-":
                try:
                    self.selected_index = values[event][0]
                except:
                    pass
                #print(self.selected_index)
            elif event =="-DRONECONFIG-":
                if not self.selected_index == None: 
                    #print(self.selected_index)
                    #print(self.drone_data)  
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
                    self.window['-STARTLOCAL-'].update(disabled=False)

                    if self.anim_file_loaded and not self.file_load_cb == None:
                        self.file_load_cb(self.anim_file_location)
            elif event == '-STARTLOCAL-':
                self.start_local_cb()
                self.start_localisation=True
                self.window['-INIT-'].update(disabled=False)
            elif event == '-UPDATE-':
                self.update1()
            elif event == '-INIT-':
                self.initialised = True
                self.init_cb()
                self.window['-START-'].update(disabled=True)
                self.window['-DRONECONFIG-'].update(disabled=True)
        if not self.data_thread == None:
            while True:
                if self.data_thread.is_alive():
                    print("data thread is alive")
                else:
                    break
                pass

        self.window.close()
        if not self.shutdown:
            self.close_cb()

        self.shutdown_completed = True

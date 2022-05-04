import threading
import PySimpleGUI as sg
import time

count = 0

#Show Controller

class ShowController():
    WINDOW = sg.Window("asd", layout=[[sg.Text("Hello World", key="-TEXT-"),sg.Button("Change", key="-Change-")],[sg.Text("None", key="-OPENFILE-"),sg.FileBrowse("Open Animation File",key="-ANIMFILE-",target="-OPENFILE-", change_submits=True)],[sg.Table(values=[], headings=["Drone Name", "X", "Y","Z", "Yaw"])]], size=(800,600))

    def __init__(self) -> None:
        self.window = ShowController.WINDOW
        self.count = 0
        self.is_init = False

    def change_text(self,key, value):
        self.window[key].update(value)

    @staticmethod
    def start(self):
        while True:
            event, values = self.window.read()
            if event == sg.WIN_CLOSED:
                break
            elif event == "-Change-":
                self.is_init = True
                #self.change_text("-TEXT-",f'count : {self.count}')
            elif event == "Submit":
                print(event, values[event])
            elif event == "-OPENFILE-":
                print(event, values[event])

window = ShowController()
win_thread = threading.Thread(target=ShowController.start, args=[window])
win_thread.start()
while win_thread.is_alive():
    if window.is_init == True:
        #window.change_text("-TEXT-",f'Time : {time.time()}')
        pass
print("Exited")
import PySimpleGUI as sg
from threading import Thread
import os
# import random
import csv
import matplotlib, time

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# SuperFastPython.com
# example of returning multiple value from a thread
from time import sleep

# custom thread
# class DroneThread(Thread):
#     # constructor
#     def __init__(self):
#         # execute the base constructor
#         Thread.__init__(self)
#         # set a default values
#         self.x = None
#         self.y = None
#
#     # function executed in a new thread
#     def run(self):
#         # block for a moment
#         sleep(1)
#         # store data in an instance variable
#         self.x =
#         self.value2 = 99
#         self.value3 = False
#
# # create a new thread
# thread = CustomThread()
# # start the thread
# thread.start()
# # wait for the thread to finish
# thread.join()
# # report all values returned from a thread
# print(thread.value1)
# print(thread.value2)
# print(thread.value3)



def readCSV(csv_path):
    # filename = sg.PopupGetFile('Get required file', no_window=True, file_types=(("CSV Files", "*.csv"),))
    filename = csv_path
    # global x = 0
    # global y = 0
    # read csv
    try:
        with open(filename, "r") as infile:
            csvreader = csv.reader(infile)
            for line in csvreader:
                x1,y1 = line[0],line[1]
                print("x1,y1",x1,y1)
                return x1,y1
    except Exception as e:
        print('CSV READ ERROR')


def delete_fig_agg(fig_agg):
    fig_agg.get_tk_widget().forget()
    plt.close('all')

def drone_show(window,x,y):  # this should be called as a thread, then time.sleep() here would not freeze the GUI

    graph = window('graph')
    print(x,y)
    graph.draw_circle((x, y), 10, fill_color='blue', line_color='white')
    window.refresh()
    time.sleep(1)
    # return plt.gcf()



def LEDIndicator(key=None, radius=30):
    return sg.Graph(canvas_size=(radius, radius),
             graph_bottom_left=(-radius, -radius),
             graph_top_right=(radius, radius),
             pad=(0, 0), key=key)

def SetLED(window, key, color):
    graph = window[key]
    graph.erase()
    graph.draw_circle((0, 0), 12, fill_color=color, line_color=color)

def inSafeArea(x,y):
    if x>0 and x<800 and y>0 and y<800:
        return True
    else:
        return False


def gui():

    cwd = os.getcwd()

    AREADATAINPUT = [
        [sg.Text('Input CSV path', font=('Arial', 14)),
         sg.InputText('CSV path', key='-CSV-', do_not_clear=True, size=(30, 3)), sg.FileBrowse(initial_folder=cwd)],
        [sg.Button('START', font=('Arial', 10), size=(10, 3))]
    ]

    AREADRONESHOW = [
        [sg.Graph(
            canvas_size=(400, 400),
            graph_bottom_left=(0, 0),
            graph_top_right=(800, 800),
            key="-graph-",
            # enable_events=True,
            background_color='lightblue')]]
    AREALEFT = [
        [sg.Frame(layout=[[sg.Column(AREADATAINPUT, vertical_alignment='c')]], vertical_alignment='c', title='')],
        [sg.Frame(layout=[[sg.Column(AREADRONESHOW, vertical_alignment='c')]], vertical_alignment='c', title='')]]

    AREARIGHT = [[sg.Text('Drones Status Indicators', size=(20, 1))],
                 [sg.Text('Drone1'), LEDIndicator('_drone1_')],
                 [sg.Text('Drone2'), LEDIndicator('_drone2_')],
                 [sg.Text('Drone3'), LEDIndicator('_drone3_')],
                 [sg.Text('Drone4'), LEDIndicator('_drone4_')],
                 [sg.Button('Exit')]]

    layout = [[sg.Column(AREALEFT), sg.Column(AREARIGHT)]]
    sg.theme('Dark Blue 3')

    WINDOW = sg.Window("DroneShow", layout, margins=(100, 50))

    drone_agg = None
    while True:  # Event Loop
        event, value = WINDOW.read(timeout=400)
        if event == 'Exit' or event == sg.WIN_CLOSED:
            break
        # if value is None:
        #     break

        if event == 'START':
            try:
                csvpath = value['-CSV-']
                [x,y] = readCSV(csvpath)
                # print(x,y)
                drone_show(WINDOW,x,y)
                sleep(1)


            except Exception as e:
                print('ERROR')
        SetLED(WINDOW, '_drone1_', 'green' if inSafeArea(x1, y1) else 'red')
        SetLED(WINDOW, '_drone2_', 'green' if inSafeArea(x2, y2) else 'red')
        SetLED(WINDOW, '_drone3_', 'green' if inSafeArea(x3, y3) else 'red')
        SetLED(WINDOW, '_drone4_', 'green' if inSafeArea(x4, y4) else 'red')

# drones initial location
x1,y1 = 50,400
x2,y2 = 150,400
x3,y3 = 250,400
x4,y4 = 350,400


def main():

    thread1 = Thread(target=gui,name='main thread')
    thread1.run()

    thread1.join()

if __name__ == '__main__':
    main()

# readCSV("data/loc.csv")

# window.close()

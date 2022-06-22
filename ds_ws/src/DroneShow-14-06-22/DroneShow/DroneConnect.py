import PySimpleGUI as sg
from djitellopy import Tello
sg.theme('DarkAmber')   # Add a touch of color
# All the stuff inside your window.
layout = [  [sg.Text('SSID'), sg.InputText()],
            [sg.Text('Password'), sg.InputText()],
            [sg.Button('Ok'), sg.Button('Cancel')] ]

# Create the Window
window = sg.Window('Window Title', layout)
# Event Loop to process "events" and get the "values" of the inputs
while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        break
    if event == 'Ok':
        tello=Tello()
        tello.connect()
        tello.connect_to_wifi(str(values[0]),str(values[1]))
    
        break


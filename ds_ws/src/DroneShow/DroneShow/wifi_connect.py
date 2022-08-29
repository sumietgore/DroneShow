"""
A small script to connect Tello Drone to a wifi network. Please change the wifi ssid and passwod below.
Connect to your tello drone using WiFi and then run the script.
To reset the Tello drone into AP mode, please follow the Tello documentation.
"""

from lib.tellopy import Tello

ssid = "Your WiFi SSID"
password = "YourPassword"

tello = Tello()

tello.connect()

tello.connect_to_wifi(ssid, password)

tello.end()
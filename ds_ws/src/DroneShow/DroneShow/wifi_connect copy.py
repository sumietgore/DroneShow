from lib.tellopy import Tello

tello = Tello()

tello.connect()

tello.connect_to_wifi("MyWLAN", "9922921605")

tello.end()
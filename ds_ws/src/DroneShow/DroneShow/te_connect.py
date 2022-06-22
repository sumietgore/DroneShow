from lib.tellopy import Tello
import time

tello = Tello(host="192.168.1.100")

tello.connect()

time.sleep(3)

tello.takeoff()

time.sleep(3)

tello.land()

tello.end()
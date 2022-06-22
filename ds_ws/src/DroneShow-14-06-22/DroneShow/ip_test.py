from lib.tellopy import Tello
import time

tello = Tello(host="192.168.1.102")

tello.connect()

tello.takeoff()
time.sleep(3)
tello.land()

tello.end()
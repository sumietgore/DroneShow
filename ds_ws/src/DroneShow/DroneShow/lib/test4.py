from tellopy import Tello
import time

tello = Tello()

tello.connect()

print(f'battery : {tello.get_battery()} temp : {tello.get_temperature()}')

tello.takeoff()


for i in range(0,10):
    tello.send_rc_control(0,50,50,0)
    time.sleep(0.1)
    tello.send_rc_control(0,0,0,0)
    time.sleep(0.1)

tello.stop()

time.sleep(2)

for i in range(0,10):
    tello.send_rc_control(0,50,0,0)
    time.sleep(0.1)
    tello.send_rc_control(0,0,0,0)
    time.sleep(0.1)

tello.stop()

time.sleep(10)

tello.send_rc_control(0,0,0,0)

time.sleep(1)

tello.land()

tello.end()
from lib.tellopy import TelloSwarm, Tello

import time

swarm = TelloSwarm.fromIps(["192.168.1.100","192.168.1.101", "192.168.1.102"])

swarm.connect()
"""
v=[5,25]"""

#tello = Tello()
#tello.send_rc_control()

#tello.get_battery()

"""swarm.takeoff()"""

tellos = swarm.tellos

for tello in tellos:
    print(tello.get_battery())

"""time.sleep(2)"""

"""for tello in tellos:
    tello.land()
"""
"""swarm.parallel(lambda i, tello: tello.send_rc_control(0,v[i],0,0))

time.sleep(5)"""

"""tellos[0].send_rc_control(0,20,0,0)

#swarm.parallel(lambda i, tello: tello.send_rc_control(0,10,0,0))

time.sleep(3)

tellos[0].send_rc_control(0,0,0,0)

time.sleep(3)

swarm.land()"""

#swarm.takeoff()

#time.sleep(1)

#swarm.land()


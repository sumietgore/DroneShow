from djitellopy import TelloSwarm
import time

TelloSwarm.add_drone('172.20.10.2')
TelloSwarm.add_drone('172.20.10.4')

tellos = TelloSwarm.get_drones()
print(tellos)

swarm = TelloSwarm.start_swarm(tellos)

swarm.connect()

swarm.takeoff()

time.sleep(2)

swarm.land()

swarm.end()
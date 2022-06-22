from lib.tellopy import Tello

tello = Tello()

tello.connect()

tello.connect_to_wifi("HUAWEI-B525-CBE1", "JJJD6YQ0R88")

tello.end()
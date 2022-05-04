from djitellopy import Tello
from .State import State

class TelloController():
    def __init__(self):
        #initialise state
        self.state = State()
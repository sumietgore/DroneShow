from concurrent.futures import thread
from threading import Thread
import time

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.start()
        return thread
    return wrapper

class Test():
    def __init__(self):
        self.stop_thread = False
        self.control_thread = self.func()

    @threaded
    def func(self):
        while True:
            if self.stop_thread == True:
                break
            print(f'{time.perf_counter()}', end='\r')

if __name__ == "__main__":
    c = Test()
    time.sleep(3)
    c.stop_thread = True


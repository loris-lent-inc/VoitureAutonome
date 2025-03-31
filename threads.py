import threading
import time
from abc import ABC, abstractmethod
#from controle_accel import controle_accel
#from servo_controller import servo_controller
#from traitement_image import traitement_image
#from ultrason import ultrason


class Toolbox:
    def __init__(self):
        self.meca = None
        self.cam = None
        self.dir = None
        self.us = None
        self.watchdog = None
        
        self.isFinished = False

    def finish(self):
        if not self.isFinished:
            self.meca.finish()
            self.cam.finish()
            self.dir.finish()
            self.us.finish()
            self.isFinished = True
            
    def last_distance(self):
        return self.us.last_mesure
    
    def last_image(self):
        return self.cam.last_image
    
    def last_heading(self):
        return self.cam.curr_steering_angle
    
    def last_fps(self):
        return self.cam.last_fps
    
    def set_steering(self, angle):
        self.dir.next_steering = angle
    
    def set_accel(self, speed):
        self.meca.next_speed = speed
    
    def set_dir(self, dir):
        self.meca.setDir(dir)
    

class toolThread(threading.Thread, ABC):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.heartbeat()

    def finish(self):
        self.running = False
    
    def heartbeat(self):
        self.last_heartbeat = time.time()

        
    @abstractmethod
    def run(self):
        pass

class Watchdog(threading.Thread):
    def __init__(self, toolbox, timeout=10):
        threading.Thread.__init__(self)
        self.threads = [toolbox.meca, toolbox.cam, toolbox.us]
        self.running = True
        self.timeout = timeout

    def run(self):
        while self.running:
            current_time = time.time()
            for thread in self.threads:
                if thread == None:
                    continue
                
                elif not thread.is_alive():
                    print(f"Restarting {thread.name}")
                    thread.start()
                elif current_time - thread.last_heartbeat > self.timeout:
                    print(f"Thread {thread.name} is not responding. Restarting...")
                    thread.running = False
                    thread.join()
                    new_thread = type(thread)(thread.tool)
                    self.threads[self.threads.index(thread)] = new_thread
                    new_thread.start()
            time.sleep(1)
        
        self.finish()
import threading
import time
from abc import ABC, abstractmethod
from controle_accel import controle_accel
from controle_dir import controle_dir
from traitement_image import traitement_image
from ultrason import ultrason


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
        return self.us.tool.last_mesure
    
    def last_image(self):
        return self.cam.tool.last_image
    
    def last_heading(self):
        return self.cam.tool.curr_steering_angle
    
    def last_fps(self):
        return self.cam.tool.last_fps
    
    def set_steering(self, angle):
        self.dir.next_steering = angle
        self.dir.needs_steering = True
    
    def set_accel(self, speed):
        self.meca.next_speed = speed
    
    def set_dir(self, dir):
        self.meca.tool.setDir(dir)
    

class toolThread(threading.Thread, ABC):
    def __init__(self, tool):
        threading.Thread.__init__(self)
        self.tool = tool
        self.running = True
        self.heartbeat()

    def finish(self):
        self.running = False
        self.tool.finish()
    
    def heartbeat(self):
        self.last_heartbeat = time.time()

        
    @abstractmethod
    def run(self):
        pass

class AccelerationThread(toolThread):
    def __init__(self, tool):
        toolThread.__init__(self, tool)
        self.next_speed = 0
        
    def run(self):
        while self.running:
            try:
                self.tool.setAccel(self.next_speed)
            except KeyboardInterrupt:
                self.running = False
        
        self.finish()

class DirectionThread(toolThread):
    def __init__(self, tool):
        self.needs_steering = True
        self.next_steering = 90

    def run(self):
        while self.running:
            try:
                if(self.needs_steering):
                    self.tool.setSteering(self.next_steering)
                    self.needs_steering = False
                
                self.heartbeat()
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.running = False
        
        self.finish()

class ImageProcessingThread(toolThread):
    def run(self):
        while self.running:
            self.tool.test_video_picam()
            self.heartbeat()
            time.sleep(0.1) # pas de sleep car déjà lent
        self.finish()

class UltrasonThread(toolThread):
    def run(self):
        while self.running:
            self.tool.mesurer_distance()
            self.heartbeat()
            time.sleep(0.5)
        
        self.finish()

class WatchdogThread(threading.Thread):
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
import threading
import time
from abc import ABC, abstractmethod
from controle_vehicule import controle_vehicule
from traitement_image import traitement_image
from ultrason import ultrason


class Toolbox:
    def __init__(self, DIR_PIN, PWM_PIN, TRIGGER_PIN, ECHO_PIN):
        self.ctrl = controle_vehicule(DIR_PIN, PWM_PIN)
        self.trt = traitement_image()
        self.us = ultrason(TRIGGER_PIN, ECHO_PIN)
        
        self.ctrl_thread = None
        self.trt_thread = None
        self.us_thread = None
        self.watchdog = None
        
        self.isFinished = False

    def finish(self):
        if not self.isFinished:
            self.ctrl.finish()
            self.trt.finish()
            self.us.finish()
            self.isFinished = True

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

class ControlThread(toolThread):
    def run(self):
        self.speed = 1
        self.accel = 1
        
        while self.running:
            try:
                if self.speed > 99 or self.speed < 1:
                    self.accel *= -1
                self.speed += self.accel
                self.tool.setAccel(self.speed)
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
    def __init__(self, timeout=10):
        threading.Thread.__init__(self)
        self.threads = []
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
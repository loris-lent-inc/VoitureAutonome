import threading
import time
from abc import ABC, abstractmethod
from controle_vehicule import *
from traitement_image import *
from ultrason import *

class tools:
    def __init__(self, DIR_PIN, PWM_PIN, TRIGGER_PIN, ECHO_PIN):
        self.ctrl = controle_vehicule(DIR_PIN, PWM_PIN)
        self.trt = traitement_image()
        self.us = ultrason(TRIGGER_PIN, ECHO_PIN)
        self.isFinished = False

    def heartbeat(self):
        self.last_heartbeat = time.time()

    def finish(self):
        if not self.isFinished:
            self.ctrl.finish()
            self.trt.finish()
            self.us.finish()
            self.isFinished = True

class ComponentThread(threading.Thread, ABC):
    def __init__(self, component):
        threading.Thread.__init__(self)
        self.component = component
        self.running = True

    def finish(self):
        self.running = False
        self.component.finish()

    @abstractmethod
    def run(self):
        pass

class ControlThread(ComponentThread):
    def run(self):
        self.speed = 1
        self.accel = 1
        
        while self.running:
            try:
                if self.speed > 99 or self.speed < 1:
                    self.accel *= -1
                self.speed += self.accel
                self.component.setAccel(self.speed)
                self.heartbeat()
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.running = False
        
        self.finish()

class ImageProcessingThread(ComponentThread):
    def run(self):
        while self.running:
            self.component.test_video_picam()
            if cv2.waitKey(1) == ord('q'):
                self.component.stop()
                self.running = False
            self.heartbeat()
            #time.sleep(0.1) # pas de sleep car déjà lent
            
        self.finish()

class UltrasonThread(ComponentThread):
    def run(self):
        while self.running:
            print(self.component.mesurer_distance())
            self.heartbeat()
            time.sleep(0.5)
        
        self.finish()

class WatchdogThread(threading.Thread):
    def __init__(self, threads, timeout=5):
        threading.Thread.__init__(self)
        self.threads = threads
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
                    new_thread = type(thread)(thread.component)
                    self.threads[self.threads.index(thread)] = new_thread
                    new_thread.start()
            time.sleep(1)
        
        self.finish()
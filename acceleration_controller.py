import RPi.GPIO as GPIO
import pigpio
from threads import toolThread
import  time
from gui import DIR_PIN, PWM_PIN

class acceleration_controller(toolThread):
    def __init__(self, DIR_pin, PWM_pin):
        toolThread.__init__(self)
        self.dir_pin = DIR_pin
        self.pwm_pin = PWM_pin
        self.next_speed = 0

        GPIO.setmode(GPIO.BCM)
        #GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        self.dir = True
        self.setDir(self.dir)

        self.pwmValue = 0
        #self.pwm = GPIO.PWM(PWM_pin, 1000)
        #self.pwm.start(self.pwmValue)
        self.pwm = pigpio.pi()

    def set_PWM(self, value):
        self.pwmValue = value
        self.pwm.set_PWM_dutycycle(self.pwm_pin, value)
        #self.pwm.ChangeDutyCycle(value)
    
    def setAccel(self, value):
        if(self.pwmValue < 1 and value < 5 and value > 0):
            self.set_PWM(255)
            time.sleep(0.02)
        self.set_PWM(value)

    def setDir(self, value):
        self.dir = value
        GPIO.output(self.dir_pin, self.dir)
    
    def changeDir(self):
        self.setDir(not self.dir)

    def run(self):
        while self.running:
            try:
                self.setAccel(self.next_speed)
                self.heartbeat()
                time.sleep(0.2)
            except Exception as e:
                self.running = False

        
        self.finish()
    
    def finish(self):
        print(f"Finishing throttle tool")
        try:
            self.set_PWM(0)
        except:
            pass
        self.pwm.stop()
        self.running = False
        GPIO.cleanup()


if __name__ == "__main__":
    control = acceleration_controller(DIR_PIN, PWM_PIN)
    try:
        while True:
            for i in range(0, 101, 1):
                control.setAccel(i)
                time.sleep(0.02)
            for i in range(100, -1, -1):
                control.setAccel(i)
                time.sleep(0.02)
            control.changeDir()
            print("Loop!")
    except KeyboardInterrupt:
        control.finish()
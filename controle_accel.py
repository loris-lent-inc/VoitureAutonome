import RPi.GPIO as GPIO
import  time


class controle_accel():
    def __init__(self, DIR_pin, PWM_pin):
        self.dir_pin = DIR_pin
        self.pwm_pin = PWM_pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

        self.dir = True
        self.setDir(self.dir)

        self.pwmValue = 0
        self.pwm = GPIO.PWM(PWM_pin, 100)
        self.pwm.start(self.pwmValue)

    def setAccel(self, value):
        if(self.pwmValue < 3):
            self.pwm.ChangeDutyCycle(100)
            time.sleep(0.02)
        
        self.pwmValue = value
        self.pwm.ChangeDutyCycle(self.pwmValue)

    def setDir(self, value):
        self.dir = value
        GPIO.output(self.dir_pin, self.dir)
    
    def changeDir(self):
        self.setDir(not self.dir)

    def finish(self):
        self.pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    control = controle_accel()
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
import RPi.GPIO as GPIO
inmport picamera2 as Picamera2
import  time

DIR = 23
PWM = 18

DIR_STATE = True

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)



pwm = GPIO.PWM(PWM, 100)
pwm.start(0)

try:
    while True:
        for i in range(0, 101, 1):
            pwm.ChangeDutyCycle(i)
            time.sleep(0.02)
        for i in range(100, -1, -1):
            pwm.ChangeDutyCycle(i)
            time.sleep(0.02)
        DIR_STATE = not DIR_STATE
        GPIO.output(DIR, DIR_STATE)
        print("Loop!")
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
        
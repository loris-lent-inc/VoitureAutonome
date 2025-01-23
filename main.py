from controle_vehicule import *
from traitement_image import *
from ultrason import *

TRIGGER_PIN = 20
ECHO_PIN = 21
DIR_PIN = 18
PWM_PIN = 19

def setup():
    ctrl = controle_vehicule(DIR_PIN, PWM_PIN)
    trt = traitement_image()
    us = ultrason(TRIGGER_PIN, ECHO_PIN)
    return ctrl, trt, us

def main_loop(ctrl, trt, us):
    keep_going = True
    k_interrupt = False
    accel = 1
    speed = 1

    while keep_going:
        # mesure:
        print(us.mesurer_distance())
        
        # traitement:
        trt.test_video_picam()
        if cv2.waitKey(1) == ord('q'):
            trt.stop()
        
        # controle :
        try:
            if speed > 99 or speed < 1:
                accel *= -1
            speed += accel
            ctrl.setAccel(speed)
        except KeyboardInterrupt:
            k_interrupt = True
        
        # sleep:
        time.sleep(0.02)

        # conditionnal exit:
        if trt.exit or k_interrupt:
            ctrl.finish()
            trt.finish()
            keep_going = False
    
    return True

def finish(ctrl, trt, us):
    ctrl.finish()
    trt.finish()
    us.finish()

if __name__ == "__main__":
    ctrl, trt, us = setup()
    main_loop(ctrl, trt, us)
    finish(ctrl, trt, us)

    
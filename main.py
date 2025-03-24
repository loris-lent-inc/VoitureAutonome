#!/usr/bin/env venv/bin/python
from threads import *

TRIGGER_PIN = 20
ECHO_PIN = 21
DIR_PIN = 18
PWM_PIN = 19

MECA_ENABLE = False
CAM_ENABLE = True
US_ENABLE = False

def setup():
    control_thread = image_thread = ultrason_thread = None
    
    if MECA_ENABLE:
        control_thread = ControlThread(components.ctrl)
    if CAM_ENABLE:
        image_thread = ImageProcessingThread(components.trt)
    if US_ENABLE:
        ultrason_thread = UltrasonThread(components.us)
    
    watchdog_thread = WatchdogThread([control_thread, image_thread, ultrason_thread])
    
    threads = [control_thread, image_thread, ultrason_thread, watchdog_thread]
    return threads

def main_loop(components):
    keep_going = True
    k_interrupt = False
    while keep_going:
        # mesure:
        if US_ENABLE:
            print(us.mesurer_distance())
        
        # traitement:
        if CAM_ENABLE:
            components.trt.test_video_picam()
            if cv2.waitKey(1) == ord('q'):
                trt.stop()
        
        # controle :
        if MECA_ENABLE:
            try:
                if speed > 99 or speed < 1:
                    accel *= -1
                speed += accel
                components.ctrl.setAccel(speed)
            except KeyboardInterrupt:
                k_interrupt = True
        
        # sleep:
        time.sleep(0.02)

        # conditionnal exit:
        if components.trt.exit or k_interrupt:
            components.finish()
            keep_going = False
    
    return True


if __name__ == "__main__":
    components = tools(DIR_PIN, PWM_PIN, TRIGGER_PIN, ECHO_PIN)
    main_loop(components)
    components.finish()

    
from threads import *

TRIGGER_PIN = 20
ECHO_PIN = 21
DIR_PIN = 18
PWM_PIN = 19

def setup():
    control_thread = ControlThread(components.ctrl)
    image_thread = ImageProcessingThread(components.trt)
    ultrason_thread = UltrasonThread(components.us)
    
    watchdog_thread = WatchdogThread([control_thread, image_thread, ultrason_thread])
    
    threads = [control_thread, image_thread, ultrason_thread, watchdog_thread]
    return threads

def main_loop(components):




    while keep_going:
        # mesure:
        print(us.mesurer_distance())
        
        # traitement:
        components.trt.test_video_picam()
        if cv2.waitKey(1) == ord('q'):
            trt.stop()
        
        # controle :
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

    
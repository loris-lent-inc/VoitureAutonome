#!/usr/bin/env venv/bin/python
from threads import *
from gui import GUI

TRIGGER_PIN = 20
ECHO_PIN = 21
DIR_PIN = 18
PWM_PIN = 19

MECA_ENABLE = False
CAM_ENABLE = True
US_ENABLE = False

def setup():
    toolbox = Toolbox(DIR_PIN, PWM_PIN, TRIGGER_PIN, ECHO_PIN)
    
    if MECA_ENABLE:
        toolbox.ctrl_thread = ControlThread(toolbox.ctrl)
    if CAM_ENABLE:
        toolbox.trt_thread = ImageProcessingThread(toolbox.trt)
    if US_ENABLE:
        toolbox.us_thread = UltrasonThread(toolbox.us)
    
    toolbox.watchdog = WatchdogThread([toolbox.ctrl_thread, toolbox.trt_thread, toolbox.us_thread])
    #tools_threads = [control_thread, image_thread, ultrason_thread, toolbox_watchdog]
    
    print("Finished setting up threads and components")
    return toolbox

def main_loop(toolbox):
    keep_going = True
    k_interrupt = False
    
    if MECA_ENABLE:
        toolbox.ctrl_thread.start()
    if CAM_ENABLE:
        toolbox.trt_thread.start()
    if US_ENABLE:
        toolbox.us_thread.start()
    toolbox.watchdog.start()
    
    while keep_going:
        # mesure:
        if US_ENABLE:
            print(toolbox.us.last_mesure)
        
        # traitement:
        if CAM_ENABLE:
            toolbox.trt.affiche_image()
        
        # controle :
        if MECA_ENABLE:
            try:
                if speed > 99 or speed < 1:
                    accel *= -1
                speed += accel
                toolbox.ctrl.setAccel(speed)
            except KeyboardInterrupt:
                k_interrupt = True
        
        # sleep:
        time.sleep(0.5)

        # conditionnal exit:
        if toolbox.trt.exit or k_interrupt:
            tools.finish()
            keep_going = False
    
    return True


if __name__ == "__main__":
    my_app = GUI()
    my_app.mainloop()
    toolbox = setup()
    #main_loop(toolbox)
    #toolbox.finish()

    
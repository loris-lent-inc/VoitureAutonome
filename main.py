#!/usr/bin/env venv/bin/python
from gui import GUI, TRIGGER_PIN, PWM_PIN, DIR_PIN, STR_PIN, ECHO_PIN
from threads import Toolbox, Watchdog
from acceleration_controller import *
from servo_controller import *
from traitement_image import *
from ultrason import *
import cv2
import sys

is_headless = False

def setup_and_start(meca_state='on', cam_state='on', us_state='on', 
                    dir_pin=DIR_PIN, pwm_pin=PWM_PIN, str_pin=STR_PIN, 
                    trig_pin=2, echo_pin=3):
    """Setup and start components with provided parameters or defaults"""
    
    # If running from GUI, lock it
    if is_headless:
        print("Running in headless mode with default values")
    elif 'my_app' in globals() and hasattr(my_app, 'lock'):
        my_app.lock()
        # Get values from GUI
        if hasattr(my_app, 'meca_state'):
            meca_state = my_app.meca_state.get()
        if hasattr(my_app, 'cam_state'):
            cam_state = my_app.cam_state.get()
        if hasattr(my_app, 'us_state'):
            us_state = my_app.us_state.get()
        if hasattr(my_app, 'dir_spin') and hasattr(my_app.dir_spin, 'var'):
            dir_pin = int(my_app.dir_spin.var.get())
        if hasattr(my_app, 'pwm_spin') and hasattr(my_app.pwm_spin, 'var'):
            pwm_pin = int(my_app.pwm_spin.var.get())
        if hasattr(my_app, 'str_spin') and hasattr(my_app.str_spin, 'var'):
            str_pin = int(my_app.str_spin.var.get())
        if hasattr(my_app, 'trig_spin') and hasattr(my_app.trig_spin, 'var'):
            trig_pin = int(my_app.trig_spin.var.get())
        if hasattr(my_app, 'echo_spin') and hasattr(my_app.echo_spin, 'var'):
            echo_pin = int(my_app.echo_spin.var.get())
    
    toolbox = Toolbox()
    
    toolbox.watchdog = Watchdog(toolbox)
    if meca_state == 'on':
        toolbox.meca = acceleration_controller(dir_pin, pwm_pin)
        toolbox.dir = servo_controller(str_pin, angle_min=60, angle_max=140)
    if cam_state == 'on':
        toolbox.cam = traitement_image()
    if us_state == 'on':
        toolbox.us = ultrason(3,2)
    
    print(f"MECA:{meca_state} ; CAM: {cam_state} ; US: {us_state}")
    print("Finished setting up threads and components")

    main_loop(toolbox, meca_state, cam_state, us_state)
    return toolbox

def main_loop(toolbox, meca_state='on', cam_state='on', us_state='on'):
    # Start threads
    if meca_state == 'on':
        toolbox.meca.start()
        toolbox.dir.start()
    if cam_state == 'on':
        toolbox.cam.start()
    if us_state == 'on':
        toolbox.us.start()
    
    # Starting watchdog
    toolbox.watchdog.start()
    
    # Set variables
    keep_going = True
    k_interrupt = False
    distance = 50
    image = []
    fps = 0
    angle = 0
    speed = accel = 5
    start = time.time()
    while keep_going:
        # mesure:
        if us_state == 'on':
            distance = toolbox.last_distance()
        
        # traitement:
        if cam_state == 'on':
            image = toolbox.last_image()
            angle = toolbox.last_heading()
            fps = toolbox.last_fps()
            affiche_image(image)
        
        # controle :
        if meca_state == 'on':
            try:
                if (time.time() - start) < 15:
                   speed = 0
                elif distance > 30 or distance < 0:
                    speed = 0
                else:
                    speed = 0
                toolbox.set_accel(speed)
                toolbox.set_steering(angle)
            except KeyboardInterrupt:
                k_interrupt = True
        
        print(f"Distance: {distance:.1f}cm;\tAngle: {angle:.1f};\tFPS: {fps:.1f};\tSpeed: {speed}")
        
        # sleep:
        time.sleep(0.5)

        # conditionnal exit:
        if k_interrupt:
            toolbox.finish()
            keep_going = False
    
    return True

def affiche_image(image = [], title = "Detection d'objets"):
    if is_headless or len(image) == 0:
        return
    cv2.namedWindow(title)
    cv2.imshow(title,image )
    cv2.waitKey(1)

if __name__ == "__main__":
    if not is_headless:
        try:
            # Try to initialize and run the GUI
            my_app = GUI()
            my_app.run_button.configure(command=setup_and_start)
            my_app.mainloop()
        except Exception as e:
            # If GUI fails, run in headless mode with default values
            is_headless = True  # Passer en mode headless
            print(f"GUI initialization failed: {e}")
    
    if is_headless:
        setup_and_start(us_state='on')

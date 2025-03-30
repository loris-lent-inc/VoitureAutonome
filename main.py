#!/usr/bin/env venv/bin/python
from gui import GUI
from threads import *
import cv2

def setup_and_start():
    my_app.lock()
    toolbox = Toolbox()
    
    toolbox.watchdog = WatchdogThread(toolbox)
    if my_app.meca_state.get() == 'on':
        toolbox.meca = AccelerationThread(controle_accel(int(my_app.dir_spin.var.get()), int(my_app.pwm_spin.var.get())))
        toolbox.dir = DirectionThread(controle_dir(int(my_app.str_spin.var.get())))
    if my_app.cam_state.get() == 'on':
        toolbox.cam = ImageProcessingThread(traitement_image())
    if  my_app.us_state.get() == 'on':
        toolbox.us = UltrasonThread(ultrason(int(my_app.trig_spin.var.get()), int(my_app.echo_spin.var.get())))
    
    print(f"MECA:{my_app.meca_state.get()} ; CAM: {my_app.cam_state.get()} ; US: {my_app.us_state.get()}")
    print("Finished setting up threads and components")

    main_loop(toolbox)
    return toolbox

def main_loop(toolbox):
    # Start threads
    if my_app.meca_state.get() == 'on':
        toolbox.meca.start()
    if my_app.cam_state.get() == 'on':
        toolbox.cam.start()
    if my_app.us_state.get() == 'on':
        toolbox.us.start()
    
    # Starting watchdog
    toolbox.watchdog.start()
    
    # Set variables
    keep_going = True
    k_interrupt = False
    distance = 0
    image = []
    fps = 0
    angle = 0
    speed = accel = 1
    while keep_going:
        # mesure:
        if my_app.us_state.get() == 'on':
            distance = toolbox.last_distance()
        
        # traitement:
        if my_app.cam_state.get() == 'on':
            image = toolbox.last_image()
            angle = toolbox.last_heading()
            fps = toolbox.last_fps()
            affiche_image(image)
        
        # controle :
        if my_app.meca_state.get() == 'on':
            try:
                if speed > 254 or speed < 1:
                    accel *= -1
                speed+=accel
                toolbox.set_accel(speed)
            except KeyboardInterrupt:
                k_interrupt = True
        
        #print(f"Distance: {distance:.1f}cm;\tAngle: {angle:.1f};\tFPS: {fps:.1f};\tSpeed: {speed}")
        
        # sleep:
        time.sleep(0.001)

        # conditionnal exit:
        if k_interrupt:
            tools.finish()
            keep_going = False
    
    return True

def affiche_image(image = [], title = "Detection d'objets"):
    if len(image) == 0:
        return
    cv2.namedWindow(title)
    cv2.imshow(title,image )
    cv2.waitKey(1)

if __name__ == "__main__":
    my_app = GUI()
    my_app.run_button.configure(command=setup_and_start)
    my_app.mainloop()
    
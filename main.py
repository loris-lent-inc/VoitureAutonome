#!/usr/bin/env venv/bin/python
from gui import GUI
from threads import Toolbox, Watchdog
from acceleration_controller import *
from servo_controller import *
from traitement_image import *
from ultrason import *
import cv2

def setup_and_start():
    my_app.lock()
    toolbox = Toolbox()
    
    toolbox.watchdog = Watchdog(toolbox)
    if my_app.meca_state.get() == 'on':
        toolbox.meca = acceleration_controller(int(my_app.dir_spin.var.get()), int(my_app.pwm_spin.var.get()))
        toolbox.dir = servo_controller(int(my_app.str_spin.var.get()), angle_min=60, angle_max=140)
    if my_app.cam_state.get() == 'on':
        toolbox.cam = traitement_image()
    if  my_app.us_state.get() == 'on':
        toolbox.us = ultrason(int(my_app.trig_spin.var.get()), int(my_app.echo_spin.var.get()))
    
    print(f"MECA:{my_app.meca_state.get()} ; CAM: {my_app.cam_state.get()} ; US: {my_app.us_state.get()}")
    print("Finished setting up threads and components")

    main_loop(toolbox)
    return toolbox

def main_loop(toolbox):
    # Start threads
    if my_app.meca_state.get() == 'on':
        toolbox.meca.start()
        toolbox.dir.start()
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
    speed = accel = 5
    start = time.time()
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
                if(time.time() - start) < 5:
                    toolbox.set_accel(100)
                    toolbox.set_steering(60)
                else:
                    toolbox.set_accel(steering(angle))
                    toolbox.set_steering(110)
            except KeyboardInterrupt:
                k_interrupt = True
        
        print(f"Distance: {distance:.1f}cm;\tAngle: {angle:.1f};\tFPS: {fps:.1f};\tSpeed: {speed}")
        
        # sleep:
        time.sleep(0.5)

        # conditionnal exit:
        if k_interrupt:
            tools.finish()
            keep_going = False
    
    return True

def steering(angle):
    vis_min = -90
    vis_max = 90
    steer_min = 60
    steer_max = 150
    return steer_min + ((steer_max-steer_min) * (angle - vis_min) / (vis_max - vis_min))

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
    
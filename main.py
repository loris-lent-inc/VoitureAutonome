#!/usr/bin/env venv/bin/python
from gui import GUI
from threads import *
import cv2

def setup_and_start():
    my_app.lock()
    
    toolbox = Toolbox(int(my_app.dir_spin.var.get()), int(my_app.pwm_spin.var.get()), int(my_app.trig_spin.var.get()), int(my_app.echo_spin.var.get()))
    print(f"MECA:{my_app.meca_state.get()} ; CAM: {my_app.trt_state.get()} ; US: {my_app.us_state.get()}")
    
    toolbox.watchdog = WatchdogThread()
    if my_app.meca_state.get() == 'on':
        toolbox.ctrl_thread = ControlThread(toolbox.ctrl)
        toolbox.watchdog.threads.append(toolbox.ctrl_thread)
    if my_app.trt_state.get() == 'on':
        toolbox.trt_thread = ImageProcessingThread(toolbox.trt)
        toolbox.watchdog.threads.append(toolbox.trt_thread)
    if  my_app.us_state.get() == 'on':
        toolbox.us_thread = UltrasonThread(toolbox.us)
        toolbox.watchdog.threads.append(toolbox.us_thread)
    
    print("Finished setting up threads and components")
    
    main_loop(toolbox)
    return toolbox

def main_loop(toolbox):
    # Start threads
    if my_app.meca_state.get() == 'on':
        toolbox.ctrl_thread.start()
    if my_app.trt_state.get() == 'on':
        toolbox.trt_thread.start()
    if my_app.us_state.get() == 'on':
        toolbox.us_thread.start()
    
    # Starting watchdog
    toolbox.watchdog.start()
    
    # Set variables
    keep_going = True
    k_interrupt = False
    distance = 0
    image = []
    fps = 0
    angle = 0
    while keep_going:
        # mesure:
        if my_app.us_state.get() == 'on':
            distance = toolbox.us.last_mesure
        
        # traitement:
        if my_app.trt_state.get() == 'on':
            image, fps, angle = toolbox.trt.current_results
            affiche_image(image)
        
        # controle :
        if my_app.meca_state.get() == 'on':
            try:
                if speed > 99 or speed < 1:
                    accel *= -1
                speed += accel
                toolbox.ctrl.setAccel(speed)
            except KeyboardInterrupt:
                k_interrupt = True
        
        print(f"Distance: {distance:.1f}cm ; Angle: {angle} ; FPS: {fps:.1f}")
        
        # sleep:
        time.sleep(0.5)

        # conditionnal exit:
        if toolbox.trt.exit or k_interrupt:
            tools.finish()
            keep_going = False
    
    return True

def affiche_image(image = [], title = "Detection d'objets"):
    if len(image) == 0:
        return
    cv2.namedWindow(title)
    cv2.imshow(title,image )
    cv2.waitKey(1)
    #print("shown")


if __name__ == "__main__":
    my_app = GUI()
    my_app.run_button.configure(command=setup_and_start)
    my_app.mainloop()
    #toolbox = setup()
    #main_loop(toolbox)
    #toolbox.finish()

    
from spinbox import gui, Spinbox

TRIGGER_PIN = 3
ECHO_PIN = 2
STR_PIN = 12
DIR_PIN = 16
PWM_PIN = 13

class GUI(gui.CTk):
    def __init__(self):
        super().__init__()
        self.title("Smart Haul")
        self.geometry("500x820")
        self.locked = False
        
        self.cam_state = gui.StringVar(value='on')
        self.us_state = gui.StringVar(value='on')
        self.meca_state = gui.StringVar(value='on')
        
        self.cam_checkbox = gui.CTkCheckBox(self, text="Image acquistion+analysis", variable=self.cam_state, onvalue='on', offvalue='off')
        self.cam_checkbox.grid(row=0, column=0, padx=20, pady=(10,20), sticky='w')
        self.us_checkbox = gui.CTkCheckBox(self, text="Ultrasound proximity sensor", variable=self.us_state, onvalue='on', offvalue='off')
        self.us_checkbox.grid(row=1, column=0, padx=20, pady=(0,20), sticky='w')
        self.meca_checkbox = gui.CTkCheckBox(self, text="Movement of the vehicle", variable=self.meca_state, onvalue='on', offvalue='off')
        self.meca_checkbox.grid(row=2, column=0, padx=20, pady=(0,20), sticky='w')

        self.trig_spin = Spinbox(self, "Trigger PIN :", TRIGGER_PIN, 0, 40)
        self.trig_spin.grid(row=3, column=0, columnspan=3, padx=20, pady=20)
        self.echo_spin = Spinbox(self, "Echo PIN :", ECHO_PIN, 0, 40)
        self.echo_spin.grid(row=4, column=0, columnspan=3, padx=20, pady=20)
        self.dir_spin = Spinbox(self, "Forward/Backward PIN :", DIR_PIN, 0, 40)
        self.dir_spin.grid(row=5, column=0, columnspan=3, padx=20, pady=20)
        self.pwm_spin = Spinbox(self, "Throttle PIN :", PWM_PIN, 0, 40)
        self.pwm_spin.grid(row=6, column=0, columnspan=3, padx=20, pady=20)
        self.str_spin = Spinbox(self, "Steering PIN :", STR_PIN, 0, 40)
        self.str_spin.grid(row=7, column=0, columnspan=3, padx=20, pady=20)
        
        self.run_button = gui.CTkButton(self, text="Run!", command='')
        self.run_button.grid(row=8, column=1, padx=20, pady=20)
        
    def lock(self):
        self.locked = True
        self.cam_checkbox.configure(state='disabled')
        self.us_checkbox.configure(state='disabled')
        self.meca_checkbox.configure(state='disabled')
        self.trig_spin.configure(state='disabled')
        self.echo_spin.configure(state='disabled')
        self.dir_spin.configure(state='disabled')
        self.pwm_spin.configure(state='disabled')
        self.str_spin.configure(state='disabled')
        
    def unlock(self):
        self.locked = False
        self.cam_checkbox.configure(state='normal')
        self.us_checkbox.configure(state='normal')
        self.meca_checkbox.configure(state='normal')
        self.trig_spin.configure(state='normal')
        self.echo_spin.configure(state='normal')
        self.dir_spin.configure(state='normal')
        self.pwm_spin.configure(state='normal')
        self.str_spin.configure(state='normal')
    
    def toogle_lock(self):
        if(self.locked):
            self.unlock()
        else:
            self.lock()                
            
        

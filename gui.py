import customtkinter as gui

class GUI(gui.CTk):
    def __init__(self):
        super().__init__()
        self.title("Smart Haul")
        self.geometry("640x720")

        trt_checkbox = gui.CTkCheckBox(self, text="Image acquistion+analysis")
        trt_checkbox.grid(row=0, column=0, padx=20, pady=(10,20), sticky='w')
        us_checkbox = gui.CTkCheckBox(self, text="Ultrasound proximity sensor")
        us_checkbox.grid(row=1, column=0, padx=20, pady=(0,20), sticky='w')
        meca_checkbox = gui.CTkCheckBox(self, text="Movement of the vehicle")
        meca_checkbox.grid(row=2, column=0, padx=20, pady=(0,20), sticky='w')

        run_button = gui.CTkButton(self, text="Run!", command='')
        run_button.grid(row=3, column=0, padx=20, pady=20)
        
        test_spin = gui.CTk
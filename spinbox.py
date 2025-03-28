import customtkinter as gui

class Spinbox(gui.CTkFrame):
    def __init__(self, parent, label_text="PIN:", default_value=0, from_=0, to=9999):
        super().__init__(parent)
        
        self.var = gui.StringVar(value=default_value)

        # Label
        self.label = gui.CTkLabel(self, text=label_text)
        self.label.grid(row=0, column=0, columnspan=3, pady=(0,5))
        
        self.spinbox = gui.CTkEntry(
            self,
            textvariable=self.var
        )
        self.spinbox.grid(row=1, column=1, padx=10, pady=10)

        # Ajout de boutons pour incrémenter et décrémenter la valeur
        self.increase_button = gui.CTkButton(self, text="+", command=self.increase)
        self.increase_button.grid(row=1, column=2)

        self.decrease_button = gui.CTkButton(self, text="-", command=self.decrease)
        self.decrease_button.grid(row=1, column=0)

        self.from_ = from_
        self.to = to

    def increase(self):
        current_value = int(self.var.get())
        if current_value < self.to:
            self.var.set(current_value + 1)

    def decrease(self):
        current_value = int(self.var.get())
        if current_value > self.from_:
            self.var.set(current_value - 1)

    def get_value(self):
        return self.var.get()

    def set_value(self, value):
        self.var.set(value)
        
    def configure(self, state=None):
        if state != None:
            self.increase_button.configure(state=state)
            self.decrease_button.configure(state=state)
            self.spinbox.configure(state=state)

# Exemple d'utilisation
if __name__ == "__main__":
    gui.set_appearance_mode("System")  # Modes: "System" (default), "Dark", "Light"
    gui.set_default_color_theme("blue")  # Themes: "blue" (default), "green", "dark-blue"

    root = gui.CTk()
    spinbox = Spinbox(root, default_value=1234)
    spinbox.pack()
    root.mainloop()

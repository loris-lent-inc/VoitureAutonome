import customtkinter as ctk

class Spinbox(ctk.CTkFrame):
    def __init__(self, parent, default_value=0, from_=0, to=9999):
        super().__init__(parent)

        self.var = ctk.StringVar(value=default_value)

        self.spinbox = ctk.CTkEntry(
            self,
            textvariable=self.var
        )
        self.spinbox.pack(padx=10, pady=10)

        # Ajout de boutons pour incrémenter et décrémenter la valeur
        self.increase_button = ctk.CTkButton(self, text="+", command=self.increase)
        self.increase_button.pack(side="right")

        self.decrease_button = ctk.CTkButton(self, text="-", command=self.decrease)
        self.decrease_button.pack(side="left")

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

# Exemple d'utilisation
if __name__ == "__main__":
    ctk.set_appearance_mode("System")  # Modes: "System" (default), "Dark", "Light"
    ctk.set_default_color_theme("blue")  # Themes: "blue" (default), "green", "dark-blue"

    root = ctk.CTk()
    spinbox = Spinbox(root, default_value=1234)
    spinbox.pack()
    root.mainloop()

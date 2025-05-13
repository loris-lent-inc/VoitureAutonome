import pigpio
from threads import toolThread
import time
from gui import STR_PIN
def set_steering(angle):
        vis_min = 0.0
        vis_max = 180.0
        steer_min = 60.0
        steer_max = 150.0
        #steer = steer_min + (float(steer_max-steer_min) * float(angle - vis_min) / float(vis_max - vis_min))
        steer = angle - ((vis_max + vis_min)/2) + ((steer_max+steer_min)/2)
        #print(f"Steering to {steer}°")
        return steer


class servo_controller(toolThread):
    """
    Classe pour contrôler un servo-moteur connecté au Raspberry Pi
    en utilisant la bibliothèque pigpio.
    """
    
    def __init__(self, SRV_PIN, freq=50, angle_min=0, angle_max=180):
        """
        Initialiser le servo-moteur.
        
        Args:
            SRV_PIN (int): Numéro de la broche GPIO à laquelle le servo est connecté
            freq (int): Fréquence du PWM en Hz (50Hz par défaut)
            angle_min (int): Angle minimum du servo en degrés
            angle_max (int): Angle maximum du servo en degrés
        """
        toolThread.__init__(self)
        self.pin = SRV_PIN
        self.freq = freq
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.next_steering = 90
        
        # Initialisation de pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Impossible de se connecter au daemon pigpio")
        
        # Configuration de la fréquence PWM (pigpio utilise une période en μs)
        self.period = 1000000 // self.freq  # Conversion Hz en période μs
        
        # Position actuelle
        self.angle_courant = None
    
    def angle_vers_pulse_width(self, angle):
        """0..
        Convertir un angle en largeur d'impulsion (en μs).
        Pour la plupart des servos standard :
        - 500μs correspond à 0°
        - 2500μs correspond à 180°
        
        Args:
            angle (float): Angle en degrés
            
        Returns:
            int: Largeur d'impulsion en μs
        """
        # Limitation de l'angle à l'intervalle [angle_min, angle_max]
        angle = max(self.angle_min, min(self.angle_max, angle))
        
        # Conversion de l'angle en largeur d'impulsion
        return int(500 + (angle / 180.0) * 2000)
    
    def tourner(self, angle):
        """
        Faire tourner le servo-moteur à l'angle spécifié.
        
        Args:
            angle (float): Angle cible en degrés
        """
        pulse_width = self.angle_vers_pulse_width(angle)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)
        self.angle_courant = angle
        #time.sleep(0.3)  # Attendre que le servo atteigne la position
        #print(f"angle:{angle}")

    def run(self):
        while self.running:
            try:
                #if(self.needs_steering):
                angle = set_steering(self.next_steering)
                self.tourner(angle)
                    #self.needs_steering = False
                
                self.heartbeat()
                time.sleep(0.2)
            except Exception as e:
                self.running = False
        
        self.finish()

    def finish(self):
        """
        Destructeur pour s'assurer que les ressources sont libérées.
        """
        try:
            # Arrêter le servo en mettant la largeur d'impulsion à 0
            print(f"Finishing servo tool")
            self.running = False
            self.pi.set_servo_pulsewidth(self.pin, 0)
            self.pi.stop()
        except:
            pass  # Ignorer les erreurs lors du nettoyage
  

# Exemple d'utilisation
if __name__ == "__main__":
    try:
        # Créer un objet servo sur la broche GPIO 18
        servo = servo_controller(SRV_PIN=STR_PIN, freq=200, angle_min=60, angle_max=140)
        
        # Tourner à différents angles
        while True:
            #print("Tournage à 100°")
            servo.tourner(105)
            time.sleep(0.5)
            servo.tourner(70)
            time.sleep(0.5)
            servo.tourner(105)
            time.sleep(0.5)
            servo.tourner(140)
            time.sleep(0.5)
            servo.tourner(105)
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    
    finally:
        # Nettoyage
        if 'servo' in locals():
            servo.finish()

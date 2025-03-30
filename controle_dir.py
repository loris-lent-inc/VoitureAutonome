import RPi.GPIO as GPIO
import time

class controle_dir():
    """
    Classe pour contrôler un servo-moteur connecté au Raspberry Pi
    en utilisant la modulation de largeur d'impulsion (PWM).
    """
    
    def __init__(self, SRV_PIN, freq=50, angle_min=0, angle_max=180):
        """
        Initialiser le servo-moteur.
        
        Args:
            pin (int): Numéro de la broche GPIO à laquelle le servo est connecté
            freq (int): Fréquence du PWM en Hz (50Hz par défaut)
            angle_min (int): Angle minimum du servo en degrés
            angle_max (int): Angle maximum du servo en degrés
        """
        self.pin = SRV_PIN
        self.freq = freq
        self.angle_min = angle_min
        self.angle_max = angle_max
        
        # Configuration du GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
        # Création de l'objet PWM
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)  # Démarrage avec un rapport cyclique de 0%
        
        # Position actuelle
        self.angle_courant = None
    
    def angle_vers_duty_cycle(self, angle):
        """
        Convertir un angle en rapport cyclique (duty cycle).
        En général, pour un servo standard :
        - 2.5% correspond à 0°
        - 12.5% correspond à 180°
        
        Args:
            angle (float): Angle en degrés
            
        Returns:
            float: Rapport cyclique correspondant
        """
        # Limitation de l'angle à l'intervalle [angle_min, angle_max]
        angle = max(self.angle_min, min(self.angle_max, angle))
        
        # Conversion de l'angle en rapport cyclique
        # La formule peut varier selon le servo-moteur
        return 2.5 + (angle / 180.0) * 10.0
    
    def tourner(self, angle):
        """
        Faire tourner le servo-moteur à l'angle spécifié.
        
        Args:
            angle (float): Angle cible en degrés
        """
        duty_cycle = self.angle_vers_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.angle_courant = angle
        time.sleep(0.3)  # Attendre que le servo atteigne la position
    
    def balayer(self, debut, fin, pas=10, temps_attente=0.2):
        """
        Balayer le servo d'un angle à un autre par incréments.
        
        Args:
            debut (float): Angle de départ en degrés
            fin (float): Angle de fin en degrés
            pas (float): Incrément en degrés
            temps_attente (float): Temps d'attente entre chaque pas en secondes
        """
        if debut < fin:
            angles = range(int(debut), int(fin) + 1, pas)
        else:
            angles = range(int(debut), int(fin) - 1, -pas)
            
        for angle in angles:
            self.tourner(angle)
            time.sleep(temps_attente)
    
    def finish(self):
        """
        Destructeur pour s'assurer que les ressources sont libérées.
        """
        try:
            self.pwm.stop()
            GPIO.cleanup(self.pin)
        except:
            pass  # Ignorer les erreurs lors du nettoyage


# Exemple d'utilisation
if __name__ == "__main__":
    try:
        # Créer un objet servo sur la broche GPIO 18
        servo = controle_dir(pin=18)
        
        # Tourner à différents angles
        print("Tournage à 0°")
        servo.tourner(0)
        time.sleep(1)
        
        print("Tournage à 90°")
        servo.tourner(90)
        time.sleep(1)
        
        print("Tournage à 180°")
        servo.tourner(180)
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    
    finally:
        # Nettoyage
        servo.finish()
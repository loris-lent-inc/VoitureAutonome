import RPi.GPIO as GPIO
import time
from threads import toolThread

class ultrason(toolThread):
    # Configuration des broches GPIO
    def __init__(self, tPin, ePin):
        toolThread.__init__(self)
        self.trigger_pin = tPin
        self.echo_pin = ePin
        GPIO.setmode(GPIO.BCM)  # Utilise la numérotation BCM
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.last_mesure = -1
    
    def mesurer_distance(self):
        # S'assurer que le Trigger est bas
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.000002)

        # Envoyer une impulsion de 10 µs sur Trigger
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)  # 10 µs
        GPIO.output(self.trigger_pin, False)

        # Mesurer le temps d'écho
        while GPIO.input(self.echo_pin) == 0:
            debut = time.time()
            #print("echo0")

        while GPIO.input(self.echo_pin) == 1:
            #print("echo1")
            fin = time.time()

        # Calculer la durée de l'écho
        duree = fin - debut
        # Calculer la distance (vitesse du son = 34300 cm/s)
        distance = (duree * 34300) / 2  # Divisé par 2 pour l'aller simple
        #print(fin, debut, duree, distance)
        self.last_mesure = distance
        return distance
    
    def run(self):
        while self.running:
            self.mesurer_distance()
            self.heartbeat()
            time.sleep(1)
        
        self.finish()
        
    def finish(self):
        GPIO.cleanup()  # Réinitialise les GPIO

if __name__ == "__main__":
    us = ultrason(6, 5)

    try:
        while True:
            distance = us.mesurer_distance()
            print(f"Distance mesurée : {distance:.2f} cm")
            time.sleep(1)  # Pause d'une seconde entre les mesures
    except KeyboardInterrupt:
        print("Arrêt du programme.")
        us.finish()
       

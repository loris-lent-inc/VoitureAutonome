#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from picamera2 import Picamera2, Preview
import cv2
import numpy as np
import time
# import matplotlib.pyplot as plt
import logging
import math
import random

class traitement_image:

    def __init__(self, size_tuple = (640, 360)):
        self.picam = Picamera2()
        config = self.picam.create_still_configuration({"size": size_tuple})
        self.picam.configure(config)
        self.picam.start()
        self.f_count = 0
        self.total_time = 0
        self.curr_steering_angle = stabilized_steering_angle = 90
        self.exit = False
        cv2.namedWindow("Detection d'objets")
        

    def affiche_image(self, frame, title = "image"):
        cv2.namedWindow(title)
        cv2.imshow(title, frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def detect_edges(self, frame, showDetec=True):  # Créé une nouvelle image avec les bordure des objets bleu
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # On transforme le format BRG en HSV pour éviter les différentes teintes de bleu due à la luminosité
        
        lower_blue = np.array([20, 100, 100])  # Bleu clair
        upper_blue = np.array([30, 255, 255])  # Bleu foncé
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)  # On ne garde que les couleurs entre le bleu clair et le bleu foncé (à noter que open CV utilise une gamme de couleur entre 0 et 180)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_cleaned = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        
        # self.img_clean = cv2.addWeighted(cv2.bitwise_and(frame, frame, mask=mask_cleaned), 1, np.zeros_like(frame), 0, 0)
        
        edg = cv2.Canny(mask_cleaned, 200, 400)  # On ne garde que les contours
        
        if (showDetec):
            cv2.imshow("mask", edg)  # On affiche la nouvelle image
            #self.afficheImage(edges, "mask")

        return edg
    
    def region_of_interest(self, edges):  # Créé une nouvelle image avec seulement les bordure du bas
        height, width = edges.shape  # Dimensions de l'image
        mask = np.zeros_like(edges)  # On crée un masque vide

        # On ne garde que le bas de l'image
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        return cropped_edges

    def detect_line_segments(self, cropped_edges):  # Transforme les bordures en segments
        # tuning min_threshold, minLineLength, maxLineGap sont déterminées empiriquement
        rho = 1  # Précision sur la distance en nombre de pixels
        angle = np.pi / 180  # Précision sur l'angle en radian (= 1 degré)
        min_threshold = 10  # nombre de votes minimal
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                        np.array([]), minLineLength=8, maxLineGap=4)

        return line_segments

    def make_points(self, frame, line):  # Renvoie les extrémités de la ligne
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bas de l'image
        y2 = int(y1 * 1 / 2)  # créé des points du milieu au bas de l'image

        # lit les coordonnées dans l'image
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, frame, line_segments):  # Créé 2 lignes à partir des segments
        """
        Cette fonction combine les segments de ligne en une ou deux lignes de voies.
        Si toutes les pentes de ligne sont < 0 : alors nous n'avons détecté que la voie de gauche.
        Si toutes les pentes de la ligne sont > 0 : alors nous n'avons détecté que la voie de droite.
        """
        lane_lines = []
        if line_segments is None:  # Si il n'y a aucun segment
            logging.info('Aucun segment de ligne détecté')
            return lane_lines

        height, width, _ = frame.shape  # On prend les dimenssion de l'image
        left_fit = []
        right_fit = []

        boundary = 1 / 3
        left_region_boundary = width * (
                    1 - boundary)  # Les segments de ligne de la voie de gauche doivent être sur les 2/3 gauches de l'écran
        right_region_boundary = width * boundary  # Les segments de ligne de la voie de droite doivent être sur les 2/3 droits de l'écran

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    logging.info(
                        'On saute un segment de ligne verticale (slope=inf) : %s' % line_segment)  # On ne veut pas les lignes verticales
                    continue
                fit = np.polyfit((x1, x2), (y1, y2),
                                1)  # Génére un polynome de degré 1 qui passe par les point de coordonnées (x1, y1) et (x2, y2)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:  # Si la pente est négative et qu'on se trouve dans la partie gauche
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)  # Moyenne des pentes de la partie gauche
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))  # On génére la ligne de la partie gauche

        right_fit_average = np.average(right_fit, axis=0)  # Moyenne des pentes de la partie droite
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))  # On génére la ligne de la partie droite

        logging.debug('lignes de voies: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

        return lane_lines

    def detect_lane(self, frame, showDetec=True):  # Fonction qui résume les précédentes

        self.edges = self.detect_edges(frame, showDetec)
        cropped_edges = self.region_of_interest(self.edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)
        return lane_lines

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=2):
        line_image = np.zeros_like(frame)  # On créé une image de la taille de l'image de base
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)  # On représente les 2 lignes
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  # On supperpose les 2 images
        return line_image

    def compute_steering_angle(self, frame, lane_lines):
        """ 
        Trouver l'angle de braquage basé sur les coordonnées de la ligne de la voie.
        Nous supposons que la caméra est calibrée pour pointer vers le point mort.
        """
        if len(lane_lines) == 0:  # Si on ne voit pas de ligne on ne fait rien
            logging.info('Aucune ligne de couloir détectée, ne rien faire')
            return 90

        height, width, _ = frame.shape
        if len(lane_lines) == 1:  # Si on voit une ligne, on suit sa direction
            logging.debug('On n a détecté qu une seule ligne de couloir, il faut juste la suivre. %s' % lane_lines[0])
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.0  # 0.0 signifie que la caméra pointe vers le centre, -0.03 : la caméra est centrée sur la gauche, +0.03 signifie que la caméra pointe vers la droite
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid  # Décalage du x du bout de la ligne à suivre par rapport au milieu

        # Trouver l'angle de direction, qui est l'angle entre la direction de la navigation et l'extrémité de la ligne centrale.
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (en radian) par rapport à la ligne verticale centrale
        angle_to_mid_deg = int(
            angle_to_mid_radian * 180.0 / math.pi)  # angle (en degrés) par rapport à la ligne verticale centrale
        steering_angle = angle_to_mid_deg + 90  # C'est l'angle de braquage nécessaire à la roue avant de la voiture

        logging.debug('nouvel angle de braquage: %s' % steering_angle)
        return steering_angle

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        try:
            height, width, _ = frame.shape
        except:
            height, width = frame.shape

        # déterminer la ligne de cap à partir de l'angle de braquage
        # la ligne de direction (x1, y1) est toujours au centre du bas de l'écran
        # (x2, y2) nécessite un peu de trigonométrie

        # Note : l'angle de direction de :
        # 0-89 degré : tourner à gauche
        # 90 degrés : aller tout droit
        # 91-180 degré : tourner à droite 
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        if steering_angle_radian == 0:
            x2 = x1
        else:
            x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))  # = x1 + x_offset
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

    def stabilize_steering_angle(self, new_steering_angle, num_of_lane_lines,
                                max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
        """
        Utilisation du dernier angle de braquage pour stabiliser l'angle de braquage
        si le nouvel angle est trop différent de l'angle actuel, 
        tourner seulement de max_angle_deviation degrés
        """
        if num_of_lane_lines == 2:  # si les deux voies sont détectées, alors on peut dévier davantage
            max_angle_deviation = max_angle_deviation_two_lines
        else:  # Si une seule voie est détectée, ne déviez pas trop.
            max_angle_deviation = max_angle_deviation_one_lane

        angle_deviation = new_steering_angle - self.curr_steering_angle
        if abs(angle_deviation) > max_angle_deviation:  # Si l'angle de déviation est trop important on l'augmente juste du maximum
            stabilized_steering_angle = int(self.curr_steering_angle
                                            + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = new_steering_angle
        return stabilized_steering_angle

    def steering_angle(self, frame):
        lane_lines = self.detect_lane(frame)
        steering_angle = self.compute_steering_angle(frame, lane_lines)

        return steering_angle

    def get_steering(self):
        return self.curr_steering_angle
    
    def test_video_picam(self):
        try:
            show = False
            self.start_time = time.perf_counter()

            # Capture de la frame
            acq = self.picam.capture_array()
            frame = cv2.rotate(cv2.cvtColor(acq, cv2.COLOR_RGB2BGR), cv2.ROTATE_180)

            # Détection de voies
            lane_lines = self.detect_lane(frame, show)
            if len(lane_lines)>0:
                new_steering_angle = self.compute_steering_angle(frame, lane_lines)
                # Stabilisation
                stabilized_steering_angle = self.stabilize_steering_angle(
                    new_steering_angle,
                    num_of_lane_lines=len(lane_lines)
                )
                self.curr_steering_angle = stabilized_steering_angle

            # Génération de l'image finale
            heading_image = self.display_heading_line(self.edges, self.curr_steering_angle, (255,255,255))

            # Mesures de performance
            self.end_time = time.perf_counter()
            
            self.p_time = self.end_time - self.start_time
            self.total_time += self.p_time
            self.f_count += 1
            avg_time = self.total_time / self.f_count
            
            fps = 1 / avg_time if avg_time > 0 else 0

            if self.f_count > 100:
                self.f_count = 0
                self.total_time = 0

            # Affichage des résultats
            cv2.imshow("Detection d'objets", heading_image)
            print(f"t = {self.p_time:.4f}s ; FPS = {fps:.2f} ; Angle = {self.curr_steering_angle:.0f}°")
        except Exception as e:
            print("Error: %s", repr(e))
            self.finish()

    def finish(self):
        self.picam.stop()
        self.exit = True

if __name__ == "__main__":
    t = traitement_image()
    
    while not t.exit:
        t.test_video_picam()
        if cv2.waitKey(1) == ord('q'):
            t.finish()
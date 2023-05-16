import cv2 as cv
import imutils
from typing import Tuple, List
import numpy as np
import math

from .preprocessing import preprocessing_img
from .pieces_detection_v3 import PiecesDetector
from .piece import Piece

class PiecesIdentifier:
    def __init__(self, img, size, pieces=[], preprocess=True, verbose=False, visualize=False):
        """Inicializar detector de piezas

        Args:
            img (Mat): Imagen
            size (float): Area de la imagen total en pixeles
            pieces (List[Piece], optional): Piezas que ya han sido previamente detectadas con algun otro algoritmo. Defaults to [].
            preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to True.
            verbose (bool, optional): Mo"  + strar mensajes de seguimiento. Defaults to False.
            visualize (bool, optional): Visualizar imagenes intermedias. Defaults to False.
        """
        self.img = img
        self.size = size
        self.preprocess = preprocess
        self.verbose = verbose
        self.visualize = visualize
        
        self.pieces = pieces
        self.PIECE_WIDTH_MM = 19
        self.PIECE_HEIGHT_MM = 38
        
        self.processed_img = self.__preprocess_img(img)

    def __preprocess_img(self, img):
        if self.preprocess:
            return preprocessing_img(img, visualize=False)
        else:
            return img
    
    def __piece_recognition(self, piece, img, copy_img=True):
        """Reconocimiento de pieza que ha sido aislada a partir de una mascara

        Args:
            piece: Pieza a reconocer
            img: Imagen para visualizar contornos.
            copy_img (bool, True): indica si se crea una nueva imagen o se modifica la original.

        Returns:
            Piece: Pieza identificada.
        """
        if copy_img:
            img_i = img.copy()
        else:
            img_i = img
        # Preprocesamiento de la imagen
        masked = cv.bitwise_and(self.processed_img, piece.mask)
        
        # Contornos internos a la pieza
        _,orig_contours, _ = cv.findContours(masked, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
        ref_min = round(6e-5*self.processed_img.size,2)
        if self.verbose: print("Area minima de referencia para descartar ruido: "+  str(ref_min))
        filtered_contours = [contour for contour in orig_contours if cv.contourArea(contour) > ref_min]
        
        # Diferenciar entre los puntos y la linea separadora
        # Si no hay contornos internos (salvo el de la propia pieza), entonces la pieza esta del reves
        if len(filtered_contours) <= 1:
            piece.type = "reves"
            if self.verbose: print("Pieza de tipo  "  + str(piece.type))
            if self.visualize: self.__visualize_piece_contours(img_i, piece)
            return piece
        
        dot_contours = []
        line_contours = [] # En principio solo deberia detectar un contorno, pero ponemos varios para mo"  + strar si hay errores
        if self.verbose: print("Hay "  + str(len(filtered_contours) - 1)+" contornos internos")
        
        # Medimos el area de la pieza para tenerla como referencia
        piece_area = cv.contourArea(filtered_contours[0])
        if self.verbose: print("Area pieza grande: "  + str(piece_area))
        
        if self.visualize:
            # Miramos los contornos internos a la pieza para obtener sus contornos circulares o rectangulares
            for contour in filtered_contours[1:]:
                inner_area = cv.contourArea(contour[0])
                # Solo debe encontrar una pieza
                if inner_area > 0.9*piece_area and inner_area < 1.1*piece_area:
                    continue
                center_i, size_i, angle_i = cv.minAreaRect(contour)
                box = np.int64(cv.boxPoints((center_i, size_i, angle_i)))
                ratio = size_i[1]/size_i[0]
                inner_area = size_i[0]*size_i[1]
                
                if ratio > 0.5 and ratio < 2 and inner_area  < 0.05*piece_area:
                    dot_contours.append(box) # punto
                elif inner_area < 0.07*piece_area:
                    line_contours.append(box) # linea separadora
                    
                if self.verbose: print("Area interna: "  + str(round(inner_area,2)) + " Ratio: "  + str(round(ratio,2)) +" Centro: "  + str(np.round(center_i,2)))

        # Giramos la pieza para tenerlo en posicion horizontal o vertical
        if self.verbose: print("Se hace giro de "  + str(round(piece.angle, 2)))
        img_r = imutils.rotate(masked, piece.angle)
        _,rot_contours, _ = cv.findContours(img_r, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
        ref_min = 0.01*piece_area
        if self.verbose: print("area minima de referencia: "  + str(ref_min))
        filtered_contours = [contour for contour in rot_contours if cv.contourArea(contour) > ref_min]
            
        if self.verbose: 
            # Comprobamos si esta en vertical u horizontal -> ahora mismo nos
            (x,y,w,h) = cv.boundingRect(filtered_contours[0])
            is_vertical = h/w > 1
            print("Pieza vertical: "  + str(is_vertical))
            print("Hay "  + str(len(filtered_contours) - 1)+ " contornos internos")
            
        piece_area = cv.contourArea(filtered_contours[0])
        if self.verbose: print("Area pieza grande: "  + str(piece_area))
        
        # Obtenemos las posiciones de la linea separadora y de los puntos
        ref = (0,0)
        dots = []
        for contour in filtered_contours[1:]:
            (x,y,w,h) = cv.boundingRect(contour)
            ratio = h/w
            if ratio < 0.5 or ratio > 2:
                ref = (x,y)
            else:
                dots.append((x,y))
            if self.verbose: print("Area interna: "  + str(cv.contourArea(contour)) +" Ratio: "  + str(round(ratio,2)) + " Centro: "  + str(x,y))
        # Separamos los puntos en dos grupos
        # Siempre comparamos con x
        n_dots_up = len([dot for dot in dots if dot[0] < ref[0]])
        n_dots_down = len([dot for dot in dots if dot[0] > ref[0]])
        if self.verbose: print("Hay "  + str(len(dots))+" puntos. Arriba/Izquierda: "  + str(n_dots_up) +". Abajo/Derecha: "  + str(n_dots_down))

        piece.dots = [n_dots_up, n_dots_down]
        # El primer valor siempre debe ser mayor igual que el segundo
        piece.type = ""  + str(max(n_dots_up,n_dots_down)) + "x" + str(min(n_dots_up,n_dots_down))
        
        if self.verbose: print("Pieza de tipo "  + str(piece.type))
        if self.visualize: self.__visualize_piece_contours(img_i, piece, dot_contours, line_contours)
        return piece
    
    def __visualize_piece_contours(self, img_i, piece, dot_contours=[], line_contours=[]):
        cv.drawContours(img_i,[piece.contour],0,(255,0,0),thickness=2)
        for c in dot_contours:
            radius_c = round(abs(np.sqrt((c[1][0] - c[0][0])**2 + (c[1][1] - c[0][1])**2))/2)
            center_c = c[0] + (c[2] - c[0])/2
            cx = round(center_c[0])
            cy = round(center_c[1])
            cv.circle(img_i, (cx,cy), radius_c, color=(0,255,0), thickness=1)
        for l in line_contours:
            cv.drawContours(img_i,[l],0,(0,0,255),thickness=1)
        cx = round(piece.center[0])
        cy = round(piece.center[1])
        cv.rectangle(img_i, (cx-12, cy-6), (cx+12, cy+6), (255,255,255), thickness=-1)
        cv.putText(img_i, piece.type, (cx-12, cy+3), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1, cv.LINE_AA)
        cv.imshow("Reconocimiento de piezas", img_i)
    
    def pieces_recognition(self):
        """Identificacion de piezas utilizando la libreria OpenCV

        Returns:
            List[dict]: Lista de piezas reconocidas.
        """
        img_i = self.img.copy()
        if not len(self.pieces):
            detector = PiecesDetector(self.processed_img,size=self.size, preprocess=False, visualize=False, verbose=self.verbose)
            self.pieces = detector.detect_pieces()
        
        if self.verbose: print("*"*20, "Se ha iniciado el reconocimiento de piezas", "*"*20)
        
        for i, piece in enumerate(self.pieces):
            if self.verbose: print("-"*10, "Se inicia reconocimiento de pieza", "-"*10)
            self.pieces[i] = self.__piece_recognition(piece, img_i, copy_img=False)
            if self.verbose: print("-"*10, "Se finaliza reconocimiento de pieza", "-"*10)
        
        if self.verbose: print("*"*20, "Se ha finalizado el reconocimiento de piezas", "*"*20)
        
        return self.pieces
    

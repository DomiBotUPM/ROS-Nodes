import cv2 as cv
import imutils
from typing import Tuple, List
import numpy as np
import math

from .preprocessing import preprocessing_img
from .pieces_detection_v2 import PiecesDetector
from .piece import Piece

class PiecesIdentifier:
    def __init__(self, img: cv.Mat, size: float, pieces: List[Piece]=[], preprocess=True, verbose=False, visualize=False):
        """Inicializar detector de piezas

        Args:
            img (Mat): Imagen
            size (float): Area de la imagen total en píxeles
            pieces (List[Piece], optional): Piezas que ya han sido previamente detectadas con algún otro algoritmo. Defaults to [].
            preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to True.
            verbose (bool, optional): Mostrar mensajes de seguimiento. Defaults to False.
            visualize (bool, optional): Visualizar imágenes intermedias. Defaults to False.
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

    def __preprocess_img(self, img: cv.Mat):
        if self.preprocess:
            return preprocessing_img(img, visualize=False)
        else:
            return img
    
    def __piece_recognition(self, piece: Piece, img: cv.Mat, copy_img=True) -> Piece:
        """Reconocimiento de pieza que ha sido aislada a partir de una máscara

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
        orig_contours, _ = cv.findContours(masked, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
        ref_min = round(6e-5*self.processed_img.size,2)
        if self.verbose: print(f"Area minima de referencia para descartar ruido: {ref_min}")
        filtered_contours = [contour for contour in orig_contours if cv.contourArea(contour) > ref_min]
        
        # Diferenciar entre los puntos y la línea separadora
        # Si no hay contornos internos (salvo el de la propia pieza), entonces la pieza está del revés
        if len(filtered_contours) <= 1:
            piece.type = "reves"
            if self.verbose: print(f"Pieza de tipo {piece.type}")
            if self.visualize: self.__visualize_piece_contours(img_i, piece)
            return piece
        
        dot_contours = []
        line_contours = [] # En principio solo debería detectar un contorno, pero ponemos varios para mostrar si hay errores
        if self.verbose: print(f"Hay {len(filtered_contours) - 1} contornos internos")
        
        # Medimos el área de la pieza para tenerla como referencia
        piece_area = cv.contourArea(filtered_contours[0])
        if self.verbose: print(f"Area pieza grande: {piece_area}")
        
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
                    line_contours.append(box) # línea separadora
                    
                if self.verbose: print(f"Area interna: {round(inner_area,2)}. Ratio: {round(ratio,2)}. Centro: {np.round(center_i,2)}.")

        # Giramos la pieza para tenerlo en posición horizontal o vertical
        cond_angle = piece.angle < 0.95*90 or piece.angle > 1.05*piece.angle
        cond_angle = cond_angle and abs(piece.angle) > 5
        if cond_angle:
            if self.verbose: print(f"Se hace giro de {round(piece.angle, 2)}")
            img_r = imutils.rotate(masked, piece.angle)
            rot_contours, _ = cv.findContours(img_r, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
            ref_min = 0.01*piece_area
            if self.verbose: print(f"Área mínima de referencia: {ref_min}")
            filtered_contours = [contour for contour in rot_contours if cv.contourArea(contour) > ref_min]
        
        # Comprobamos si está en vertical u horizontal
        (x,y,w,h) = cv.boundingRect(filtered_contours[0])
        is_vertical = h/w > 1
        # Definimos si comparamos con x o con y
        index_pos = 0
        if is_vertical:
            index_pos = 1
            
        if self.verbose: 
            print(f"Pieza vertical: {is_vertical}")
            print(f"Hay {len(filtered_contours) - 1} contornos internos")
            
        piece_area = cv.contourArea(filtered_contours[0])
        if self.verbose: print(f"Area pieza grande: {piece_area}")
        
        # Obtenemos las posiciones de la línea separadora y de los puntos
        ref = (0,0)
        dots = []
        for contour in filtered_contours[1:]:
            (x,y,w,h) = cv.boundingRect(contour)
            ratio = h/w
            if ratio < 0.5 or ratio > 2:
                ref = (x,y)
            else:
                dots.append((x,y))
            if self.verbose: print(f"Area interna: {cv.contourArea(contour)}. Ratio: {round(ratio,2)}. Centro: {x,y}")
        # Separamos los puntos en dos grupos
        n_dots_up = len([dot for dot in dots if dot[index_pos] < ref[index_pos]])
        n_dots_down = len([dot for dot in dots if dot[index_pos] > ref[index_pos]])
        if self.verbose: print(f"Hay {len(dots)} puntos. Arriba/Izquierda: {n_dots_up}. Abajo/Derecha: {n_dots_down}")
        
        piece.dots = [n_dots_up, n_dots_down]
        # El primer valor siempre debe ser mayor igual que el segundo
        piece.type = f"{max(n_dots_up,n_dots_down)}x{min(n_dots_up,n_dots_down)}"
        
        if self.verbose: print(f"Pieza de tipo {piece.type}")
        if self.visualize: self.__visualize_piece_contours(img_i, piece, dot_contours, line_contours)
        return piece
    
    def __visualize_piece_contours(self, img_i, piece: Piece, dot_contours=[], line_contours=[]) -> None:
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
    
    def pieces_recognition(self) -> List[Piece]:
        """Identificacion de piezas utilizando la librería OpenCV

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
    
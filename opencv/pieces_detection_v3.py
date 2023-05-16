import cv2 as cv
from typing import List, Tuple
import numpy as np

from .preprocessing import preprocessing_img
from .piece import Piece


class PiecesDetector:
    def __init__(self, img, size, size_mm = 0.0, preprocess=True, verbose=False, visualize=False):
        """Inicializar detector de piezas

        Args:
            img (Mat): Imagen
            size (float): Area de la imagen total en pixeles
            preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to False.
            verbose (bool, optional): Mostrar mensajes de seguimiento. Defaults to False.
            visualize (bool, optional): Visualizar imagenes intermedias. Defaults to False.
        """
        self.img = img
        self.size = size
        self.preprocess = preprocess
        self.verbose = verbose
        self.visualize = visualize
        self.pieces = []  #: List[Piece]
        self.PIECE_WIDTH_MM = 19
        self.PIECE_HEIGHT_MM = 38
        
        self.processed_img = self.__preprocess_img(img)
        # Si nos dan el tamano fisico, podemos obtener directamente el ratio px/mm y el area de una pieza estandar en pixeles
        if size_mm > 0:
            self.ratio_px2mm = size_mm / size
            self.ref_piece_area = self.ratio_px2mm**2 * self.PIECE_WIDTH_MM * self.PIECE_WIDTH_MM
        else:
            self.ratio_px2mm = 0.0
            self.ref_piece_area = 0.0
    
    def __preprocess_img(self, img):
        img_i = img.copy()
        if self.preprocess:
            return preprocessing_img(img_i, visualize=False)
        else:
            return img_i
    
    def get_ratio_px2mm_from_piece(self, piece):
        """Obtener el ratio de conversion de pixeles a milimetros, a partir de una pieza. 

        Args:
            piece (Piece): Objeto de clase Piece

        Returns:
            float: Ratio de conversion px --> mm
        """
        width = min(piece.size[0], piece.size[1])
        return self.PIECE_WIDTH_MM / width
        
    def change_img(self, new_img, new_size):
        self.img = new_img
        self.size = new_size
    
    def detect_pieces(self):
        """Deteccion de las fichas de domino presentes en la zona

        Returns:
            List[Piece]: Lista de piezas detectadas.
        """
        if self.verbose: print("*"*20, "Se ha iniciado la deteccion de piezas", "*"*20)
        img_i = self.img.copy()
        
        # Detectar contornos
        _,contours,_ = cv.findContours(self.processed_img, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE) #contours, _
        #print(contours)
        filtered_contours = [contour for contour in contours if cv.contourArea(contour) > 1.6e-4*self.size] # Minima area para un punto
        
        # Si no hay ningun contorno minimamente grande, se finaliza la deteccion
        if len(filtered_contours) == 0:
            if self.verbose: print("No se ha detectado ningun contorno minimamente grande")
            return []
        
        # Encontrar solo los contornos que sean de piezas
        pieces = []  #: List[Piece]
        if self.ref_piece_area > 0:
            min_area_piece = 0.8*self.ref_piece_area
        else:
            min_area_piece = 7e-3*self.size
        
        if self.verbose: print("Tamano de referencia para detectar pieza: " + str(min_area_piece))
        # ########## COMIENZO DE CAMBIOS DE PABLO ##########
        for contour in filtered_contours:
            area = cv.contourArea(contour)
            # El tamano minimo para un punto
            if area < 1.6e-4*self.size:
                continue
            center, (width,height), angle = cv.minAreaRect(contour)
            ratio = width/height
            # Para que sea una pieza el ancho debe ser la mitad que el alto y debe ser al menos de un tamano concreto
            if width*height > min_area_piece: 
                if (ratio > 0.45 and ratio < 0.55):  
                    box = np.int64(cv.boxPoints((center, (width,height), angle)))
                    mask = np.zeros(self.processed_img.shape, np.uint8)
                    cv.fillPoly(mask, [box], color=(255))
                    if angle > 45: # horizontal
                        angle = angle - 90
                    else: # vertical
                        angle = angle + 90
                    pieces.append(Piece(mask, box, np.round(center,3), angle, size=(round(width,3), round(height,3))))
                    if self.verbose: print("Area del contorno: " + str(area)+ ". Area del rectangulo: " + str(round(width,1)) +"*" +str(round(height,1)) +" = " + str(round(width*height,2)))
                elif (ratio > 1.9 and ratio < 2.1): # horizontal o vertical pero con un angulo logico
                    box = np.int64(cv.boxPoints((center, (width,height), angle)))
                    mask = np.zeros(self.processed_img.shape, np.uint8)
                    cv.fillPoly(mask, [box], color=(255))
                    pieces.append(Piece(mask, box, np.round(center,3), angle, size=(round(width,3), round(height,3))))
                    if self.verbose: print("Area del contorno: " + str(area) +"  . Area del rectangulo: " + str(round(width,1)) +"*" + str(round(height,1)) +" = " + str(round(width*height,2)))
                else:
                    box = np.int64(cv.boxPoints((center, (width,height), angle)))
                    mask = np.zeros(self.processed_img.shape, np.uint8)
                    cv.fillPoly(mask, [box], color=(255))
                    pieces.append(Piece(mask, box, np.round(center,3), angle, size=(round(width,3), round(height,3))))
                    if self.verbose: print("Area del contorno: " + str(area) +". Area del rectangulo: " + str(round(width,1)) +"*" + str(round(height,1))+" = " + str(round(width*height,2)))

                # ########## FINAL DE CAMBIOS DE PABLO ##########         

                
        if self.verbose: print("N de elementos: " + str(len(filtered_contours)) + ". N de candidatos a piezas: " + str(len(pieces)))
        
        # Si no se ha encontrado ningun candidato a pieza, se finaliza la deteccion
        if len(pieces) == 0:
            return pieces
        
        if self.verbose: print("Se procede a buscar piezas anomalas")
        
        # Si no se tiene la referencia del tamano de una pieza, se calcula la media
        if self.ref_piece_area > 0:
            ref_area =self.ref_piece_area
        else:
            ref_area = round(np.mean([piece.get_area() for piece in pieces]),2)
        
        pieces_big = [p for p in pieces if p.get_area() >= 1.5*ref_area]
        pieces = [p for p in pieces if p.get_area() < 1.5*ref_area]
        
        if self.visualize: 
            for piece in pieces:
                cv.drawContours(img_i,[piece.contour],0,(255,0,0), thickness=2)
        
        # Se procede a aplicar otro algoritmo para los casos en que haya piezas juntas y las detecte como una sola
        if len(pieces_big):
            if self.ratio_px2mm == 0.0:
                self.ratio_px2mm = self.get_ratio_px2mm_from_piece(pieces[0])
                if self.verbose: print("Pieza utilizada como referencia para ratio. Ancho: " + str(min(pieces[0].size[0], pieces[0].size[1])) +". Ratio: " + str(round(self.ratio_px2mm, 2)))
            for i, piece in enumerate(pieces_big):
                if self.verbose: 
                    print("indice de pieza actual: " + str(i))
                    print("Se ha encontrado una pieza anomala de tamano: " + str(piece.get_area()) +" . Tamano de pieza de referencia: " + str(ref_area)+".")
                
                # Se separa la "pieza" detectada como una sola, en las verdaderas que habia
                split_pieces = self.split_piece(piece)
                for piece in split_pieces:
                    pieces.append(piece)
                    if self.visualize:
                        cv.drawContours(img_i,[piece.contour],0,(0,255,0), thickness=2)
        
            if self.verbose:  print("N  de elementos: " + str(len(contours))+". N  de piezas detectadas: " + str(len(pieces)))
        if self.visualize: cv.imshow("Deteccion de piezas", img_i)
        
        self.pieces = pieces
        
        if self.verbose: print("*"*20, "Se ha finalizado la deteccion de piezas", "*"*20)
        return pieces
    
    def split_piece(self, piece):
        """Division de una pieza anormalmente grande en sus respectivas internas

        Returns:
            List[Piece]: Lista de piezas detectadas.
        """
        print("splitear")

        if self.verbose: print("-"*5, "Se inicia la separacion de piezas", "-"*5)
        
        masked = cv.bitwise_and(self.processed_img, piece.mask)
        # Detectar contornos
        _, contours, _ = cv.findContours(masked, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
        filtered_contours = [contour for contour in contours if cv.contourArea(contour) > 1.6e-4*self.size] # Minima area para un punto
        # Encontrar solo las lineas separadoras de las piezas
        pieces = []
        for contour in filtered_contours:
            area = cv.contourArea(contour)
            center, (width,height), angle = cv.minAreaRect(contour)
            ratio = min(width,height)/max(width,height)
            
            # Condicion de tamano de la linea separadora si tenemos la referencia del tamano de pieza
            if self.ref_piece_area > 0:
                cond_size = width > 0.5*(self.ratio_px2mm*self.PIECE_WIDTH_MM) and width*height < 0.25*self.ref_piece_area
            else: # Si no tenemos la referencia
                cond_size = width*height < 1e-2*self.size
            
            # ########## COMIENZO DE CAMBIOS DE PABLO ##########
            # Para que sea una linea separadora el ratio debe ser muy pequeno o muy grande
            if ratio < 0.3 and cond_size:
                box = np.int64(cv.boxPoints((center, (width,height), angle)))
                mask = np.zeros(self.processed_img.shape, np.uint8)
                cv.fillPoly(mask, [box], color=(255))
                
                if self.verbose: print("Encontrada linea separadora de " + str(round(width,2))+"x" + str(round(height,2))+"=" + str(round(width*height,2)))
                # Con ayuda del ratio, se obtiene el contorno de la pieza
                if width > height: # Horizontal
                    new_width = 19/self.ratio_px2mm
                    new_height = 38/self.ratio_px2mm
                    if angle > 45: 
                        true_angle = angle - 90
                    else: # vertical
                        true_angle = angle + 90
                else: # Vertical
                    new_width = 38/self.ratio_px2mm
                    new_height = 19/self.ratio_px2mm
                    true_angle = angle
                
                box_piece = np.int64(cv.boxPoints((center, (new_width,new_height), angle)))
                new_mask = np.zeros(self.processed_img.shape, np.uint8)
                cv.fillPoly(new_mask, [box_piece], color=(255))
                pieces.append(Piece(new_mask, box_piece, np.round(center,3), true_angle, size=(round(new_width,3), round(new_height,3))))
                if self.verbose: print("Nueva pieza encontrada. Area de la linea separadora: " + str(area)+". Area de la pieza: " + str(round(new_width,1))+"*" + str(round(new_height,1))+" = " + str(round(new_width*new_height,2)))
            # ########## FIN DE CAMBIOS DE PABLO ##########
        if self.verbose: print("N  de elementos: " + str(len(filtered_contours))+". N  de piezas detectadas: " + str(len(pieces)))
        # if self.visualize: cv.imshow("Separacion de piezas en el juego", img_i)
        
        if self.verbose: print("-"*5, "Se finaliza la separacion de piezas", "-"*5)
        return pieces
    
    def locate_piece(self, piece, img=None, copy_img=True):
        """Localizar la pieza con respecto a la imagen o captura realizada

        Args:
            piece (dict): Datos de pieza. Contendra al menos el centro y el tamano en pixeles, asi como el angulo de rotacion.
            img (Mat, optional): Imagen para visualizar. Si no se indica utiliza la que tiene el detector.
            copy_img (bool): Indica si se sobreescribe sobre la imagen o se carga una nueva

        Returns:
            Tuple[float, Tuple[float,float], float]: Centro, tamano en mm, y angulo de rotacion
        """
        if img is not None:
            img_i = img
        else:
            img_i = self.img
        if copy_img:
            img_i = img_i.copy()
        if type(piece) == list:
            print(piece)
        
        center = np.round(self.ratio_px2mm * piece.center, 2)
        width = round(self.ratio_px2mm * piece.size[0], 2)
        height = round(self.ratio_px2mm * piece.size[1], 2)
        if self.visualize and img_i is not None:
            cx = round(piece.center[0])
            cy = round(piece.center[1])
            cv.rectangle(img_i, (cx-34, cy-6), (cx+34, cy+6), (255,255,255), thickness=-1)
            cv.putText(img_i, "(" + str(round(center[0],1)), " + str(round(center[1],1)))", (cx-33, cy+3), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1, cv.LINE_AA)
            cv.imshow("Localizacion de piezas", img_i)
        
        return center, (width, height), piece.angle

    def locate_pieces(self):
        """Localizar la pieza con respecto a la imagen o captura realizada
        
        Returns:
            List[Tuple[float, Tuple[float,float], float]]: Lista de localizaciones. Indica: centro en mm, (ancho,alto) en mm y angulo de rotacion en  .
        """
        if not len(self.pieces):
            return []
        img_i = self.img.copy()
        
        # Ratio de conversion px --> mm
        if self.ratio_px2mm == 0.0:
            self.ratio_px2mm = self.get_ratio_px2mm_from_piece(self.pieces[0])
            
        locations = []
        for i, piece in enumerate(self.pieces):
            location = self.locate_piece(piece, img_i, copy_img=False)
            self.pieces[i].center_mm =  location[0]
            self.pieces[i].size_mm = location[1]
            locations.append(location)
        return locations

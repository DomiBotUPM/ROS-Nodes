import numpy as np
from typing import List

class Piece:
    def __init__(self, mask, contour, center, angle, size, type=""):
        """Clase que define una pieza de domino

        Args:
            mask (np.ndarray): M ascara que contiene. Puede utilizarse para identificacion
            contour (np.ndarray): Contorno de la ficha
            center (tuple): Centro de la pieza
            angle (float):  angulo de rotacion
            size (tuple): Tamano
            type (str, optional): Tipo de pieza. Defaults to "" (cuando aun no se ha identificado la pieza).
        """
        self.mask = mask
        self.contour = contour
        self.center = center
        self.angle = angle
        self.size = size
        # Medidas en milimetros
        self.center_mm = 0.0
        self.size_mm = 0.0
        # Identificacion
        self.type = type
        self.dots = [] #: List[int]
        
    def __dir__(self):
        print("Angulo: " + str( self.angle) + ", size: " + str( self.size) + ", type: " + str( self.type) + ", center: " + str( self.center))

    def get_area(self, area_px=True): # -> float
        """Obtener el  area de la pieza, ya sea en pixeles o en milimetros

        Args:
            area_px (bool, optional): True:  area en pixeles. False:  area en milimetros. Defaults to True.

        Returns:
            float:  area de la pieza
        """
        if area_px:
            return self.size[0]*self.size[1]
        else:
            return self.size_mm[0]*self.size_mm[1]
        
    def __eq__(self, piece):
        """Comparacion con otra pieza. Para que sea la misma deben tener una rotacion y un centro similar, adem as de ser del mismo tipo.

        Args:
            piece (Piece): _description_

        Returns:
            bool: True: las piezas son iguales. False: las piezas son distintas
        """

        cond_center_x = (self.center[0] < 1.1*piece.center[0]) and (self.center[0] > 0.9*piece.center[0])
        cond_center_y = (self.center[1] < 1.1*piece.center[1]) and (self.center[1] > 0.9*piece.center[0])
        cond_ang = (self.angle < 1.1* piece.angle) and (self.angle > 0.9*piece.angle)
        cond_type = self.type == piece.type
        if cond_center_x and cond_center_y and cond_ang and cond_type:
            return True
        else:
            return False
          
    def esDoble(self):
        if self.type[0] == self.type[2]:
            return True
        else:
            return False
        
    def esVertical(self):
        if abs(self.angle - 90) < 30:
            return True
        else:
            return False

    def esHorizontal(self):
        if abs(self.angle) < 30:
            return True
        else:
            return False

    def sumaValor(self):
        return int(self.type[0]) + int(self.type[2])
    
    def __ne__(self, piece): # -> bool
        return not self.__eq__(piece)
        
    def __str__(self):
        return "Pice - angle: " + str( round(self.angle, 2)) + ", size: " + str( np.round(self.size, 1)) + ", type: " + str( self.type) + ", center: " + str( np.round(self.center,1)) + ""


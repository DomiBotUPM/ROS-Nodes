#!/usr/bin/env python3

import sys
import copy
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
import random

import os
from vision.vision_interface import DominoVision
import cv2 as cv
import sys
from vision.conversion_coordenadas import conversionCoordenadasJuego

global canal
canal=2

class valores_piezas_robot:
    def __init__(self):
    
        self.domino_vision = DominoVision(visualize=False, verbose=False)
        
        #subscribers
        self.subscriber_take_piece = rospy.Subscriber('/vision/take_piece', Bool, self.take_piece_callback, queue_size=1)
        
        #publishers
        self.publisher_take_piece = rospy.Publisher('/vision/take_piece', Bool, queue_size=1)
        self.publisher_posicion_pieza = rospy.Publisher('/vision/posicion_pieza_robar', Float64MultiArray, queue_size=10)              

        
        # angulo 90 - -3.139, -0.001, 0.377
        # angulo 0 - 3.089, -0.015, -1.214
        #init values
        
        self.posiciones_piezas = Float64MultiArray()
        self.posicion_pieza_robar = Float64MultiArray()

        
    def get_all_pieces_position(self):
    
        time.sleep(1)
        # datos generales
        width_game = 314
        height_game = 236
        # les he dado la vuelta respecto a lo que hizo JP 
        area_game = width_game*height_game
        domino_vision = DominoVision(visualize=False, verbose=False)

        # captura
        capture = cv.VideoCapture(canal)
        ret, frame = capture.read()

        # datos de captura
        # width = capture.get(cv.CAP_PROP_FRAME_WIDTH)
        # height = capture.get(cv.CAP_PROP_FRAME_HEIGHT)
        width = 640
        height = 480
        size = width*height

        detections = domino_vision.pieces_detection(frame, size, size_mm=area_game)
        recognitions = domino_vision.pieces_recognition(frame, size, pieces=detections)

        valores_piezas = []
        
        #para coger todas:
        for pieza in recognitions:
            posicion_pieza = conversionCoordenadasJuego(pieza.center_mm[0], pieza.center_mm[1], pieza.angle)
            posicion_pieza[0] = round((posicion_pieza[0]-10)/1000,4) # paso a mm
            posicion_pieza[1] = round((posicion_pieza[1]+3)/1000,4)
            valores_piezas.extend(posicion_pieza)
            if len(pieza.dots) == 2:
                valores_piezas.extend([pieza.dots[0], pieza.dots[1]])
            else:
                valores_piezas.extend([-1, -1])
        #print(valores_piezas)        
                
        #para publicar solo la posicion de la primera pieza :
        # angulo 90 - -3.139, -0.001, 0.377
        # angulo 0 - 3.089, -0.015, -1.214
        if(valores_piezas[2] == 0): #perfecto
            rx=3.089
            ry=-0.015
            rz=-1.214
        elif(valores_piezas[2] == 90): #perfecto
            #valores_piezas[0]
            valores_piezas[1]-=0.02
            rx=-3.139
            ry=-0.001
            rz=0.377
        self.posicion_pieza_robar.data=[valores_piezas[0], valores_piezas[1], 0.173 ,rx,ry,rz]
        
        self.publisher_posicion_pieza.publish(self.posicion_pieza_robar)
        self.publisher_take_piece.publish(False)
        #print(f"angulo {valores_piezas[2]}")
        #print(self.posicion_pieza_robar.data)  
        
        
        
    def take_piece_callback(self,data):
        
        if(data.data):
            self.get_all_pieces_position()

                     
if __name__ == '__main__':
    
    rospy.init_node('get_positions_to_take') 
    obj=valores_piezas_robot()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









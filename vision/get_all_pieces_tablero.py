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
from vision.conversion_coordenadas import conversionCoordenadasJuego

global canal
canal=2

class valores_piezas_tablero:
    def __init__(self):
    
        self.domino_vision = DominoVision(visualize=False, verbose=False)
                       
        
        #subscribers
        self.subscriber_obtain_all_pieces_tablero = rospy.Subscriber('/vision/obtain_all_pieces_tablero', Bool, self.obtain_all_pieces_tablero_callback, queue_size=1)
        
        #publishers
        self.publisher_valores_piezas = rospy.Publisher('/vision/all_pieces_tablero', Float64MultiArray, queue_size=10)              
   
        #init values
        
        self.all_pieces_tablero_send = Float64MultiArray()
        self.send_values = Float64MultiArray()

        
    def recognise_all_pieces(self):
        global canal
        time.sleep(1)
        capture = cv.VideoCapture(canal)
        ret, frame = capture.read()
        size = frame.shape[0]*frame.shape[1]
        detections = self.domino_vision.pieces_detection(frame, size)
        recognitions =  self.domino_vision.pieces_recognition(frame, size, pieces=detections)

        recognitions2 = self.domino_vision.ordenar_piezas(recognitions)
        
        valores_piezas=[]
        contador=0
        
        for pieza in recognitions2:
            posicion_pieza = conversionCoordenadasJuego(pieza.center_mm[0], pieza.center_mm[1], pieza.angle)
            posicion_pieza[0] = round(posicion_pieza[0]/1000,4) # paso a mm
            posicion_pieza[1] = round(posicion_pieza[1]/1000,4)
            
            #print([pieza.center[0], pieza.center[1], pieza.angle, pieza.dots[0], pieza.dots[1]]) round(pieza.center[0]/1000,4)
            array = [posicion_pieza[0], posicion_pieza[1], pieza.angle, pieza.dots[0], pieza.dots[1]]
            valores_piezas.extend(array)


        rospy.loginfo(f"valores_piezas")

        return valores_piezas #, valores_piezas_robot
    
   
    
        
    
    def obtain_all_pieces_tablero_callback(self,data):

        if(data.data):
            self.all_pieces_tablero_send.data=self.recognise_all_pieces() #self.valores_piezas_robot
            
            self.publisher_valores_piezas.publish(self.all_pieces_tablero_send)
  
            
        
                  
                     
if __name__ == '__main__':
    
    rospy.init_node('get_all_pieces_tablero') 
    obj=valores_piezas_tablero()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray

from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import time
import random

import os
from vision.vision_interface import DominoVision
import cv2 as cv



class valores_piezas_robot:
    def __init__(self):
        self.domino_vision = DominoVision(visualize=False, verbose=False)
        #esto se define por vision
        self.valores_piezas=[[True,3,5],[True,1,5],[True,0,2],[True,3,3],[True,6,4],[True,3,0],[True,0,6]]
        self.valores_piezas_robot = [False, 0, 0, 0, 0, 0, True, 0, 0,True, 0, 0, True, 0, 0, True, 0, 0, True, 0, 0,False, 0, 0,False, 0, 0,False, 0, 0,False, 0, 0,False, 0, 0]
            
        
        #subscribers
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1)
        
        #publishers
        self.publisher_valores_piezas = rospy.Publisher('/posiciones_piezas_robot', Float64MultiArray, queue_size=10)              
        self.publisher_valores_nuevas_piezas_robot = rospy.Publisher('/vision/valores_nuevas_piezas_robot', Float64MultiArray, queue_size=1)
        self.publisher_valores_todas_piezas_robot = rospy.Publisher('/vision/valores_todas_piezas_robot', Float64MultiArray, queue_size=1)
        
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)
        
        
        #init values
        
        self.valores_piezas_robot_send = Float64MultiArray()
        self.send_values = Float64MultiArray()
        
        #deberian llamarse en su momento
        #self.recognise_all_pieces()
        
        #self.recognise_new_pieces()
        
    def recognise_all_pieces(self):
        time.sleep(2)
        capture = cv.VideoCapture(2)
        ret, frame = capture.read()
        size = frame.shape[0]*frame.shape[1]
        detections = self.domino_vision.pieces_detection(frame, size)
        recognitions =  self.domino_vision.pieces_recognition(frame, size, pieces=detections)

        recognitions2 = self.domino_vision.ordenar_piezas(recognitions)
        
        valores_piezas=[0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        contador=0
        for pieza in recognitions2:
            #print([pieza.center[0], pieza.center[1], pieza.angle, pieza.dots[0], pieza.dots[1]])
            valores_piezas[contador] = True
            valores_piezas[contador+1] = pieza.dots[0]
            valores_piezas[contador+2] = pieza.dots[1]
            
            '''self.valores_piezas_robot[contador] = True
            self.valores_piezas_robot[contador+1] = pieza.dots[0]
            self.valores_piezas_robot[contador+2] = pieza.dots[1]'''
            contador = contador +  3
            
        '''for i in range(len(self.valores_piezas_robot)/3):
            
            self.valores_piezas[i][0] = True
            self.valores_piezas[i][1] = self.valores_piezas_robot[i*3+1]
            self.valores_piezas[i][2] = self.valores_piezas_robot[i*3+2]'''

        print(valores_piezas)

        return valores_piezas #, valores_piezas_robot
    
    def recognise_new_pieces(self):
    
        send_values=[random.randint(0,6),random.randint(0,6)]
        
        return send_values
        
        
    def init_callback(self,data):
        

        self.init=data.data
        #print(self.init)
        #recibe que ha terminado de recoger las fichas y se espera a reconocerlas
        #publica (1,abajo,arriba)
        if(self.init == "finish" or self.init == "FINISH"):
            #self.valores_piezas_robot = self.recognise_all_pieces()
            #hacer vision para obtener estos valores
            '''for i in range(len(self.valores_piezas)):
                self.valores_piezas_robot[i*3] = self.valores_piezas[i][0]
                self.valores_piezas_robot[i*3+1] = self.valores_piezas[i][1]
                self.valores_piezas_robot[i*3+2] = self.valores_piezas[i][2]'''
                
            self.valores_piezas_robot_send.data=self.recognise_all_pieces() #self.valores_piezas_robot
            
            self.publisher_valores_todas_piezas_robot.publish(self.valores_piezas_robot_send)
        
    
    def turn_callback(self,data):
        #self.recognise_new_pieces()
	#recibe que el robot ha robado pieza y quiere saber sus valores, publica abajo, arriba
        if(data.data == "evaluando"):
            self.valores_piezas_robot_send.data=self.recognise_all_pieces() #self.valores_piezas_robot
            
            self.publisher_valores_nuevas_piezas_robot.publish(self.valores_piezas_robot_send)
            #self.send_values.data=[random.randint(0,6),random.randint(0,6)]
            #self.publisher_valores_nuevas_piezas_robot.publish(self.send_values)
            #self.publisher_turn.publish("jugador")
            
        
                  
                     
if __name__ == '__main__':
    
    rospy.init_node('get_all_robot_pieces_values') 
    obj=valores_piezas_robot()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









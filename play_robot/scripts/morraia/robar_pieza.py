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


global contador
contador = 0
global msg

class jugada:
    def __init__(self):

        global contador
        global finish_jugada

        self.posiciones_piezas_robot = [[1.133, -0.843, 1.166, -1.876, -1.543, -2.354],[1.104, -1.057, 1.553, -2.071, -1.595, -2.466], [1.027, -1.236, 1.789, -2.061, -1.602, -2.513],[0.902, -1.458, 2.093, -2.155, -1.567, -2.622],[1.311, -0.837, 1.154, -1.879, -1.619, -2.195],[1.296, -1.052, 1.544, -2.053, -1.617, -2.205],[1.289, -1.268, 1.885, -2.189, -1.679, -2.236],   [1.439, -0.837, 1.153, -1.875, -1.599, -2.104], [1.403, -1.021, 1.479, -2.003, -1.579, -2.124],[1.395, -1.229, 1.817, -2.152, -1.59, -2.181]] #   con mas posiciones donde poner para robar

        self.valores_piezas=[[True,0,0],[True,0,0],[True,0,0],[True,0,0],[True,0,0],[True,0,0],[True,0,0]]
        
        #para saber cuantas piezas tiene
        self.contador_piezas_robot = 7

        #publishers
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        self.publisher_articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=1)
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)
        
        #subscribers
        
        self.subscriber_finish_go_to_pose = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1) 
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        self.subscriber_posicion_piezas_vision = rospy.Subscriber('/vision/posicion_piezas', String, self.posicion_piezas_callback, queue_size=1)
        
        #inicializacion de variables de posicion
        self.send_pose = Twist()
        self.posicion_pieza = Twist()
        

        self.posicion_camara_tablero= [0.094, -1.182, 0.514, -0.959, -1.578, -0.324] 
        self.posicion_camara_piezas_robot= [1.377, -1.102, 0.48, -0.917, -1.576, -0.544] 
        self.posicion_up_piezas_robot= [1.388, -0.906, 0.414, -1.089, -1.602, -0.553]
        self.posicion_intermedia= [0.869, -0.98, 0.514, -1.255, -1.593, -0.562]

        self.valores_piezas_robot_send = Float64MultiArray()
        self.send_articular = Float64MultiArray()
        
        self.posicion_cuna0 = [0.8, -1.226, 1.174, -1.539, -1.57, -1.134]
        self.posicion_cuna1 = [0.789, -1.094, 1.505, -1.999, -1.53, -1.141]
        self.posicion_cuna2 = [0.814, -1.052, 1.486, -2.067, -1.485, -1.127]   
        self.posicion_cuna3 = [0.867, -1.034, 1.194, -1.726, -1.599, -1.086]
        self.posicion_cuna4 = [0.955, -0.682, 0.623, -1.49, -1.616, -1.008]
        self.posicion_cuna5 = [1.01, -0.744, 0.383, -1.239, -1.584, -0.9]
        
        
        
        
           
            
    def turn_callback(self, data):
        global turno
        turno = data.data
        if(data.data == "robot" or data.data == "ROBOT"):
        
            if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
        
                #posiciones donde vans la piezas del robot
                
                self.posicion_pieza_robot = self.elegir_posicion_pieza() # self.create_twist(self.posiciones_piezas_robot[2])
                

                #posiciones de seguridad de la pieza en el tablero
                array_posicion_pieza_robar_mas_alto=[self.posicion_pieza_robar.linear.x, self.posicion_pieza_robar.linear.y, self.posicion_pieza_robar.linear.z+0.05, self.posicion_pieza_robar.angular.x, self.posicion_pieza_robar.angular.y, self.posicion_pieza_robar.angular.z]
                
                self.posicion_pieza_robar_mas_alto = self.create_twist(array_posicion_pieza_robar_mas_alto)

                
                #creacion de trayectoria
                
                self.trayectoria_jugada = (self.posicion_camara_tablero, 'open',  self.posicion_pieza_robar_mas_alto , self.posicion_pieza_robar, 'close', self.posicion_pieza_robar_mas_alto, self.posicion_cuna0, self.posicion_cuna1, self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_cuna5, self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'open', self.posicion_up_piezas_robot, self.posicion_intermedia,self.posicion_camara_tablero)
                
                    #mandar un first True de movimiento
                self.publisher_finish_go_to_pose.publish(True)
                    
                '''else:
                
                     self.trayectoria_jugada = ('open')
                     #mandar un first True de movimiento
                     self.publisher_finish_go_to_pose.publish(True) #poner robar robot'''
                        
                #ya no tiene un pieza a la vista
                self.pieza_detectada = False
                
                

                
                #publicacion de las posiciones y valores del robot 
                              
                #asignacion de los valores de las piezas - mejor pasarlo a jugada
                '''
                self.valores_piezas_robot[contador_piezas*3] = True
                self.valores_piezas_robot[contador_piezas*3+1] = self.valor_izquierda_arriba
                self.valores_piezas_robot[contador_piezas*3+2] = self.valor_derecha_abajo
                
                self.valores_piezas_robot_send.data = self.valores_piezas_robot'''
                
                #publicar los valores de las piezas
                
                #self.publisher_finish.publish(True)

                #contador_piezas= contador_piezas+1
                
                
            else:
                print("Falta pieza")





    
    def jugada_callback(self, data):


        global finish_jugada
        finish_jugada= data.data
        




    def pose_callback(self, data):

        global contador
        global finish_jugada
        global turno
                
        if(turno == "robot" or turno == "ROBOT"):
            
            if(contador< len(self.trayectoria_jugada)):
                
                if (data.data == True):
                       
                        if(isinstance(self.trayectoria_jugada[contador], str)): #si es open o close
                            if(str(self.trayectoria_jugada[contador]) == "open"):
                                self.publisher_gripper.publish("open")
                                print("open")
                            else:
                                self.publisher_gripper.publish("close")
                                print("close")
                                
                        elif("Twist" in str(type(self.trayectoria_jugada[contador]))): #enviar la pose a la que tiene que ir el robot
                            self.send_pose=self.trayectoria_jugada[contador]
                            print("pose")
                            
                            self.publisher_pose.publish(self.send_pose)
                            
                        else: #enviar la posicion articular a la que tiene que ir el robot
                            print("Articular")
                            self.send_articular.data= self.trayectoria_jugada[contador]
                            self.publisher_articular.publish(self.send_articular)           
                            

                        contador = contador +1
                        
            else:
 
                print("Tengo " + str(self.contador_piezas_robot) + " fichas")
                self.publisher_turn.publish("jugador")
                self.publisher_jugada.publish(True)
                contador=0

        else:  
            contador=0
        
    def posicion_piezas_callback(self, data):
	
        print(data)
        
        #poner las posiciones obtenidas de la vision
                
        self.posicion_pieza_robar = self.create_twist([0.264, 0.346, 0.173, 3.089, -0.015, -1.214]) #([-0.229, 0.365, 0.176, -3.116, -0.013, 1.947])
        
        self.pieza_detectada = True
        
        
        
        
    def create_twist(self,valores):
        new_Twist = Twist()
        new_Twist.linear.x=valores[0]
        new_Twist.linear.y=valores[1]
        new_Twist.linear.z=valores[2]
       
        new_Twist.angular.x=valores[3]
        new_Twist.angular.y=valores[4]
        new_Twist.angular.z=valores[5]
        
        return new_Twist
        
            
            
            
    def elegir_posicion_pieza(self):
        self.flag=False
        for i in range(len(self.valores_piezas)):
            if(self.valores_piezas[i][0] == False):  #es False, es decir, ahi no hay pieza, se puede colocar ahi:
            
                self.valores_piezas[i][0]= True
                self.flag= True
                #self.contador_piezas_robot = self.contador_piezas_robot + 1
                break	
                    
        
        self.contador_piezas_robot = self.contador_piezas_robot +1
        
        
        if(self.flag == True):
            print("Pongo la ficha en la posicion " + str(i+1) + " "  + str(self.valores_piezas))
            print(self.valores_piezas)
            return self.posiciones_piezas_robot[i]
            
        else:
            self.valores_piezas.append([True,0,0])
            print(self.valores_piezas)
            return self.posiciones_piezas_robot[i+1]
            
            
        
            
        
        
                     
                     
if __name__ == '__main__':
    
    rospy.init_node('jugada') 
    obj=jugada()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









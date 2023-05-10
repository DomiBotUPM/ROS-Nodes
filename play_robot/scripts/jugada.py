#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
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

        
        #self.posiciones_piezas_robot= ([0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[-0.036, 0.53, 0.176, 3.122, 0.01, -1.105],[0.009, 0.4, 0.172, -3.12, 0.011, 1.999], [-0.013, 0.476, 0.173, -3.101, 0.037, 1.929], [-0.048, 0.517, 0.172, 3.103, -0.007, 2.015])
        
        self.posiciones_piezas_robot= ( [0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[0.076, 0.302, 0.174, -3.07, 0.028, 1.886], [-0.007, 0.521, 0.171, 3.109, -0.026, 1.89],[-0.007, 0.461, 0.171, 3.109, -0.026, 1.89],[-0.007, 0.401, 0.171, 3.109, -0.026, 1.89], [-0.057, 0.521, 0.171, 3.109, -0.026, 1.89])
        
        self.valores_piezas=[[True,3,5],[True,1,5],[True,0,2],[True,3,3],[True,6,4],[True,3,0],[True,0,6]]
        
        #self.valores_piezas=[[True,1,0],[True,2,3],[True,3,5],[True,4,6],[True,4,4],[True,0,0],[True,3,3]]
        
        #movimientos:
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish_go_to_pose = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)
        
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        
        #evaluar los valores de las piezas
        self.subscriber_init = rospy.Publisher('/play_robot/init', String, self.finish_collect_callback,  queue_size=10)
        
        #inicio y final de jugada
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
	
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1) 
	    
	    #turno:
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)

        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        
	    #vision:
        self.subscriber_posicion_piezas_vision = rospy.Subscriber('/vision/posicion_piezas', String, self.posicion_piezas_callback, queue_size=1)
        
        #inicializacion
        self.send_pose = Twist()
        self.posicion_pieza = Twist()
        
        self.posicion_pieza_elegida = Twist()
        self.posicion_extremo_elegido = Twist()
        self.posicion_camara_tablero= self.create_twist([0.307, 0.192, 0.372, -3.119, -0.022, -1.215])
        self.posicion_camara_piezas_robot= self.create_twist([-0.077, 0.452, 0.33, -3.071, 0.028, 0.448])
        self.posicion_up_piezas_robot=self.create_twist([-0.027, 0.474, 0.314, -3.13, 0.014, 1.979]) 
        self.posicion_intermedia=self.create_twist([0.106, 0.382, 0.35, -3.063, -0.035, 0.021])
        
        
    def finish_collect_callback(self, data):
        #mover la camara a la zona del robot para evaluar los valores de las fichas
        print(data)
        if(data.data == "finish" or data.data == "FINISH"):
            self.trayectoria_jugada = ("open", self.posicion_camara_robot , "open")
            self.publisher_finish_go_to_pose.publish(True)
            
            #self.valores_piezas=[[True,1,0],[True,2,3],[True,3,5],[True,4,6],[True,4,4],[True,0,0],[True,3,3]]
            print(self.valores_piezas)
        else:
            print("recogiendo")
            
            
    def turn_callback(self, data):
        global turno
        turno = data.data
        if(data.data == "robot" or data.data == "ROBOT"):
        
            if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
        
                #posiciones donde vans la piezas del robot
                
                self.posicion_pieza_robot = self.elegir_pieza() # self.create_twist(self.posiciones_piezas_robot[2])
                

                #posiciones de seguridad
                #pieza del robot
                array_posicion_pieza_robot_mas_alto=[self.posicion_pieza_robot.linear.x, self.posicion_pieza_robot.linear.y, self.posicion_pieza_robot.linear.z +0.05, self.posicion_pieza_robot.angular.x, self.posicion_pieza_robot.angular.y, self.posicion_pieza_robot.angular.z]
                
                self.posicion_pieza_robot_mas_alto = self.create_twist(array_posicion_pieza_robot_mas_alto)
                
                #tableto
                array_posicion_pieza_extremo_mas_alto=[self.posicion_pieza_extremo.linear.x, self.posicion_pieza_extremo.linear.y, self.posicion_pieza_extremo.linear.z+0.05, self.posicion_pieza_extremo.angular.x, self.posicion_pieza_extremo.angular.y, self.posicion_pieza_extremo.angular.z]
                
                self.posicion_pieza_extremo_mas_alto = self.create_twist(array_posicion_pieza_extremo_mas_alto)

                
                #creacion de trayectoria

                #self.trayectoria_jugada = ("open", self.posicion_pieza_mas_alto, self.posicion_pieza, "close",self.posicion_pieza_mas_alto,self.posicion_cuna0, self.posicion_cuna1, self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_cuna5  ,self.posicion_pieza_robot_mas_alto, self.posicion_pieza_robot, "open", self.posicion_pieza_robot_mas_alto, self.posicion_camara)

                self.trayectoria_jugada = (self.posicion_camara_tablero, self.posicion_intermedia , 'open', self.posicion_up_piezas_robot, self.posicion_pieza_robot_mas_alto, self.posicion_pieza_robot, 'close', self.posicion_pieza_robot_mas_alto,self.posicion_up_piezas_robot,self.posicion_intermedia,  self.posicion_camara_tablero , self.posicion_pieza_extremo_mas_alto , self.posicion_pieza_extremo, 'open' ,self.posicion_pieza_extremo_mas_alto,  self.posicion_camara_tablero)
                #ya no tiene un pieza a la vista
                self.pieza_detectada = False
                
                #probar a mandar un first True de movimiento
                self.publisher_finish_go_to_pose.publish(True)

                
                #publicacion de las posiciones y valores del robot 
                #self.valores_piezas[contador_piezas]=[True,self.valor_izquierda_arriba,self.valor_derecha_abajo]
                
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
                print("Falta detectar pieza")
                   
                   
                   
                
        
        
        #self.trayectoria_jugada = ("open", self.posicion_pieza_elegida, "close", self.posicion_extremo_elegido,"open" ,self.posicion_camara)
            
    
                
                
    
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
                        else: #enviar la pose a la que tiene que ir el robot           
                            self.send_pose=self.trayectoria_jugada[contador]
                            print("pose")
                            
                            self.publisher_pose.publish(self.send_pose)

                        contador = contador +1
                        
            else:

                self.publisher_turn.publish("jugador")
                self.publisher_jugada.publish(True)
                contador=0


            #else:
               #contador=0
        else:  
            contador=0
        
    def posicion_piezas_callback(self, data):
	
        print(data)
        #poner las posiciones obtenidas de la vision
        
        
        self.posicion_pieza_extremo = self.create_twist([0.364, 0.073, 0.177, -3.141, -0.06, -1.161])
        
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
        
            
    def elegir_pieza(self):
        #falta orientacion para ponerlo alineado segun las piezas
        valor_extremo = random.randint(0,6)
        print("valor extremo: " +str(valor_extremo))
        pieza=random.randint(0,6)
        flag = False
        
        for i in range(len(self.valores_piezas)):
            if(self.valores_piezas[i][0]):  #es true, es decir, aun no se ha usado:
                if(self.valores_piezas[i][1] == valor_extremo or self.valores_piezas[i][2]  == valor_extremo ):
                    self.valores_piezas[i][0]= False
                    flag= True
                    break
        print(self.valores_piezas)
        
        if(flag != True):
            print("No tengo pieza valida")
        
        return self.create_twist(self.posiciones_piezas_robot[i])
        '''
        for i in range(len(self.valores_piezas)):
            if(valores_piezas[i][0]):  #es true, es decir, aun no se ha usado:
                if(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo1):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo1
                    
                    break
                elif(valores_piezas[i][1] == self.valor_derecha_abajo_extremo1):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo1
                    
                    break
                elif(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo2):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo2
                    
                    break
                elif(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo2):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo2
                    
                    break
                elif(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo1):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo1
                    
                    break
                elif(valores_piezas[i][1] == self.valor_derecha_abajo_extremo1):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo1
                    
                    break
                elif(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo2):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo2
                    
                    break
                elif(valores_piezas[i][1] == self.valor_izquierda_arriba_extremo2):
                    self.posicion_pieza_elegida = self.posiciones_piezas_robot[i]
                    self.posicion_extremo_elegido= self.posicion_pieza_extremo2
                    
                    break
                else: # no hay pieza para usar: robar
                    print("Esta pieza no vale")   '''
                    
        
                     
if __name__ == '__main__':
    #global msg
    rospy.init_node('poses_and_gripper') #_node
    #jugada_poses  = ([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14])
    obj=jugada()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









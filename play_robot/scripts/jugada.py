#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from decision_jugada.logica import *

from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import time
import random


global contador
contador = 0
global msg

global turno
turno = None


class jugada:
    def __init__(self):
        self.node_name ="jugada"
        global contador
        global turno
        turno = None
        
        self.posiciones_piezas_robot = [[1.226, -0.63, 0.779, -1.719, -1.634, -2.349], [1.165, -0.884, 1.261, -1.958, -1.595, -2.35],[1.132, -1.062, 1.555, -2.039, -1.624, -2.376], [1.376, -0.662, 0.857, -1.788, -1.657, -2.163], [1.337, -0.86, 1.205, -1.887, -1.631, -2.214], [1.302, -1.081, 1.593, -2.06, -1.634, -2.222], [1.479, -0.557, 0.616, -1.594, -1.609, -2.088], [1.476, -0.853, 1.205, -1.915, -1.631, -2.069],[1.458, -1.054, 1.535, -2.032, -1.631, -2.069]]

        self.valores_piezas = [[False, 0, 0], [False, 0, 0], [False, 0, 0],[False, 0, 0], [False, 0, 0], [False, 0, 0], [False, 0, 0]] #,[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0]]

        ############# publishers #############
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        self.publisher_articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=1)
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)
        self.publisher_init = rospy.Publisher('/play_robot/init', String, queue_size=10)
        
        #para obtener los valores de las piezas que se recogen
        
        #pedir info de la posicion de la pieza cuando se vaya a robar
        self.publisher_take_piece = rospy.Publisher('/vision/take_piece', Bool, queue_size=1) 
        self.publisher_obtain_all_pieces_tablero = rospy.Publisher('/vision/obtain_all_pieces_tablero', Bool, queue_size=1) 
        
        ############# subscribers #############
        #info de la pos de la pieza para robar
        self.subscriber_posicion_pieza_robar = rospy.Subscriber('/vision/posicion_pieza_robar', Float64MultiArray, self.posicion_pieza_robar_callback, queue_size=10)       
        self.publisher_valores_nuevas_piezas_robot = rospy.Subscriber('/vision/valores_nuevas_piezas_robot', Float64MultiArray, self.obtain_new_value_piece ,  queue_size=1)
        self.publisher_valores_todas_piezas_robot = rospy.Subscriber('/vision/valores_todas_piezas_robot', Float64MultiArray, self.obtain_value_all_pieces,  queue_size=1)
        	
        #info de la pos donde dejar pieza para jugar
        self.subscriber_posicion_pieza_jugar = rospy.Subscriber('/vision/all_pieces_tablero', Float64MultiArray, self.all_pieces_tablero_callback, queue_size=10)
        self.subscriber_finish_go_to_pose = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)      
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
  
        
        ############## inicializacion de variables de posicion ########
        self.send_pose = Twist()
        self.posicion_pieza = Twist()
        

        self.posicion_camara_tablero = [0.221, -1.144, 0.165, -0.594, -1.565, 4.555] #[0.094, -1.182, 0.514, -0.959, -1.578, -0.324] 
        self.posicion_camara_tablero_robar = [0.221, -1.144, 0.165, -0.594, -1.565, 4.555]
        self.posicion_camara_piezas_robot = [1.265, -0.998, 0.433, -0.946, -1.6, -2.253] #[1.377, -1.102, 0.48, -0.917, -1.576, -0.544] 
        self.posicion_up_piezas_robot = [1.388, -0.906, 0.414, -1.089, -1.602, -0.553]
        self.posicion_intermedia = [0.869, -0.98, 0.514, -1.255, -1.593, -0.562]

        self.valores_piezas_robot_send = Float64MultiArray()
        self.send_articular = Float64MultiArray()
        
        #para saber cuando gana
        self.contador_piezas_robot = 0
        
        self.posicion_cuna0 = [0.8, -1.226, 1.174, -1.539, -1.57, -1.134]
        self.posicion_cuna1 = [0.789, -1.094, 1.505, -1.999, -1.53, -1.141]
        self.posicion_cuna2 = [0.814, -1.052, 1.486, -2.067, -1.485, -1.127]   
        self.posicion_cuna3 = [0.867, -1.034, 1.194, -1.726, -1.599, -1.086]
        self.posicion_cuna4 = [0.955, -0.682, 0.623, -1.49, -1.616, -1.008]
        self.posicion_cuna5 = [1.01, -0.744, 0.383, -1.239, -1.584, -0.9]
        
        
        self.iterador_pieza_nueva=0
        
        #inicializacion de las fichas del tablero para cuando la visión lo llame
        #self.fichas_tablero=[]
        
    #cuando recoge las 7:   
    def obtain_value_all_pieces(self, data):
        valores=data.data
        for i in range(int(len(valores)/3)):
            if (valores[i*3] == 1):
                self.valores_piezas[i][0] = True
            else:   
                self.valores_piezas[i][0] = False
            self.valores_piezas[i][1] = int(valores[i*3+1])
            self.valores_piezas[i][2] = int(valores[i*3+2])
            
        rospy.loginfo(f"{self.node_name }: Los valores de las piezas recogidas: {self.valores_piezas}")
        #print("Los valores de las piezas recogidas: " + str(self.valores_piezas))
        self.trayectoria_jugada = ("open", self.posicion_intermedia, self.posicion_camara_tablero)
        self.contador_piezas_robot = len(self.valores_piezas)
        #publicar para que vaya a la posicion de vision del tableto  
        self.publisher_init.publish("collocation")            
        #self.publisher_finish_go_to_pose.publish(True)
        
        self.tipo_jugada = "colocar_camara"
        #self.publisher_turn.publish("jugador")
        
            
    #obtener los valores de las piezas que se roban
    def obtain_new_value_piece(self, data):
        valores=data.data
        contador=0
        #print(valores)
        #actualizar la pieza nueva
        #flag_new_piece=False
        
        for i in range(len(self.valores_piezas)):
            if(i==self.iterador_pieza_nueva ):
                #flag_new_piece=True
                break
            elif(self.valores_piezas[i][0]):
                contador=contador+1
        #probar si es verdad esto   
        self.valores_piezas[self.iterador_pieza_nueva][0] = True #valores[contador*3]
        self.valores_piezas[self.iterador_pieza_nueva][1] = valores[contador*3+1]
        self.valores_piezas[self.iterador_pieza_nueva][2] = valores[contador*3+2]
        
        
        rospy.loginfo(f"{self.node_name }: Los valores de las piezas tras robar: {self.valores_piezas}") 
        #print("Los valores de las piezas tras robar: " + str(self.valores_piezas))

        self.trayectoria_jugada = ("open", self.posicion_intermedia, self.posicion_camara_tablero)

        #publicar para que vaya a la posicion de vision del tablero              
        self.publisher_finish_go_to_pose.publish(True)
        #ir a la posicion de la camara sobre el tablero
        self.publisher_init.publish("collocation")
        
        #obtener los valores de todas las piezas que se han recogido
        
    
        

    # actuar cuando es el turno del robot        
    def turn_callback(self, data):
        global turno
        turno = data.data
        #es el turno del robot -  hace una foto al tablero y evalua la logica
        if(data.data == "robot" or data.data == "ROBOT"):
            self.publisher_obtain_all_pieces_tablero.publish(True)
            rospy.loginfo(f"{self.node_name }: Esperando al reconocimiento de piezas")
            #print("Esperando al reconocimiento de piezas")
            '''if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
        
                #ya no tiene un pieza a la vista
                self.pieza_detectada = False
                
                self.trayectoria_jugada = self.elegir_jugada()
                
                self.publisher_finish_go_to_pose.publish(True)

                
                
            else:
                print("Falta detectar pieza")'''




    #realizacion de la trayectoria
    def pose_callback(self, data):

        global contador
        global turno
                
        if(turno == "robot" or turno == "ROBOT"):
            
            if(contador< len(self.trayectoria_jugada)):
                
                if (data.data == True):
                       
                        if(isinstance(self.trayectoria_jugada[contador], str)): #si es open o close
                            if(str(self.trayectoria_jugada[contador]) == "open"):
                                self.publisher_gripper.publish("open")
                                #print("open")
                            elif(str(self.trayectoria_jugada[contador]) == "close"):
                                self.publisher_gripper.publish("close")
                                #print("close")

                            '''elif(str(self.trayectoria_jugada[contador]) == "robar_pieza"):
                                self.publisher_take_piece.publish(True) #pedir que reconozca una pieza'''
                                
                        elif("Twist" in str(type(self.trayectoria_jugada[contador]))): #enviar la pose a la que tiene que ir el robot
                            self.send_pose=self.trayectoria_jugada[contador]
                            #print("pose")
                            
                            self.publisher_pose.publish(self.send_pose)
                            
                        else: #enviar la posicion articular a la que tiene que ir el robot
                            #print("Articular")
                            self.send_articular.data= self.trayectoria_jugada[contador]
                            self.publisher_articular.publish(self.send_articular)           
                            

                        contador = contador +1
                        
            else: #ha acabado el moviento
            
                #comprobar si el robot ha ganado:
                if(self.contador_piezas_robot == 0):
                    rospy.loginfo(f"{self.node_name }: El robot ha ganado")
                    #print("El robot ha ganado")
                    self.publisher_turn.publish("victoria_robot")
                    
                    
                #la partida sigue
                else:
                    
                    #print("Me quedan " + str(self.contador_piezas_robot) + " fichas")
                    
                    #cuando ha robado llama a la vision para reconocer su nueva pieza
                    if(self.tipo_jugada == "robar_pieza"): # se ha robado
                        #mandar a reconocer la posicion de la pieza que debe robar
                        self.tipo_jugada = "pieza_robada"
                        #self.contador_piezas_robot = self.contador_piezas_robot + 1
                        self.publisher_turn.publish("evaluando")
                        #self.publisher_take_piece.publish(True)
                        '''elif(self.tipo_jugada == "pieza_robada"):
                        #mandar a reconocer los valores de la nueva pieza 
                        self.publisher_turn.publish("evaluando")'''
                        
                    else: # se ha jugado
                    
                        self.publisher_turn.publish("jugador")
                        rospy.loginfo(f"{self.node_name }: Mis piezas: {self.valores_piezas}")
                        #print("Mis piezas: " + str(self.valores_piezas))
                    
                    #self.publisher_jugada.publish(True)
                    
                    rospy.loginfo(f"{self.node_name }: Me quedan {self.contador_piezas_robot} fichas")
                    
                contador=0

        else:  
            contador=0
    
    
    def posicion_pieza_robar_callback(self, data):
        global turno

        #es el turno del robot -  hace una foto al tablero y evalua la logica
        if(turno == "robot" or turno == "ROBOT"):
            #print(data)

            #elegir la posicion donde dejar la pieza
            self.posicion_pieza_robot = self.elegir_posicion_pieza_robot()
            #posicion de la pieza que debe robar
            self.posicion_pieza_robar = self.create_twist(data.data)

            array_posicion_pieza_robar_mas_alto=[self.posicion_pieza_robar.linear.x, self.posicion_pieza_robar.linear.y, self.posicion_pieza_robar.linear.z+0.05, self.posicion_pieza_robar.angular.x, self.posicion_pieza_robar.angular.y, self.posicion_pieza_robar.angular.z]
                    
            self.posicion_pieza_robar_mas_alto = self.create_twist(array_posicion_pieza_robar_mas_alto)
            #recoger la pieza
            self.trayectoria_jugada=('open',self.posicion_pieza_robar_mas_alto, self.posicion_pieza_robar, 'close ', 
                                     self.posicion_pieza_robar_mas_alto, self.posicion_cuna0, self.posicion_cuna1, 
                                     self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_cuna5, 
                                     self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'open',  
                                     self.posicion_up_piezas_robot,self.posicion_camara_piezas_robot)
            #para cuando acabe sepa que debe reconocer la pieza
            self.tipo_jugada = "pieza_robada"
            #iniciar el movimiento
            self.publisher_finish_go_to_pose.publish(True)
    
    
    #detecta todas las piezas que hay en el tablero y llama a la logica
    def all_pieces_tablero_callback(self, data):
        #recibe las posiciones y valores de todas las fichas jugadas
        rospy.loginfo(f"{self.node_name }: {data.data}")
        #print(data.data)
        piezas=data.data
        contador=0
        fichas_tablero=[]
        
        for i in range(int(len(piezas)/5)):
            array=[piezas[contador+0],piezas[contador+1],piezas[contador+2],piezas[contador+3],piezas[contador+4]]
            fichas_tablero.append(array)
            contador=contador+5
        print(f"Fcihas del tableto: {fichas_tablero}")
        #ya tenemos las piezas modo [[x,y,ang,v1,v2],[x,y,ang,v1,v2],...] ahora se llama a la logica
        self.logica_juego(fichas_tablero)
        
        
        #self.posicion_pieza_jugar = self.create_twist([0.364, 0.073, 0.177, -3.141, -0.06, -1.161])
        
        
    
    '''def posicion_piezas_callback(self, data):
	
        #print(data)
        
        #poner las posiciones obtenidas de la vision
                
        self.posicion_pieza_jugar = self.create_twist([0.364, 0.073, 0.177, -3.141, -0.06, -1.161])
        
        self.posicion_pieza_robar = self.create_twist([0.264, 0.346, 0.173, 3.089, -0.015, -1.214])
        
        self.pieza_detectada = True'''
        
        
    def logica_juego(self,fichas_tablero):
    
        ficha_robot, x_dest, y_dest, ang_dest = logica(fichas_tablero, self.valores_piezas)
        
        
        self.trayectoria_jugada = self.elegir_jugada(ficha_robot, x_dest, y_dest, ang_dest)
                
        self.publisher_finish_go_to_pose.publish(True)
                
                
        #self.posicion_pieza_jugar = self.create_twist([0.364, 0.073, 0.177, -3.141, -0.06, -1.161])
           
        
    
    def elegir_jugada(self,ficha_robot, x_dest, y_dest, ang_dest):
        #
        
        self.tipo_jugada = ""
        
        
        if(ficha_robot != -1): #jugar
        
            self.tipo_jugada = "jugar"
            rospy.loginfo(f"{self.node_name }: Cojo la ficha {ficha_robot+1}")
            #print(f"Cojo la ficha {ficha_robot+1}")# + str(i+1))# + " "  + str(self.valores_piezas))
            
            #coger la ficha del robot
            self.posicion_pieza_robot = self.posiciones_piezas_robot[ficha_robot]
            
            ##evaluar el ang_dest
            
            rospy.loginfo(f"{self.node_name }: La ficha {ficha_robot+1} la pongo en la posicion {x_dest}, {y_dest}")
            
            # se quita la ficha de la lista
            self.valores_piezas[ficha_robot][0]= False
            self.contador_piezas_robot = self.contador_piezas_robot - 1
            
            #donde colocar la pieza
            self.posicion_pieza_jugar = self.create_twist([x_dest, y_dest, 0.173, -3.141, -0.06, -1.161])
            
            array_posicion_pieza_jugar_mas_alto=[self.posicion_pieza_jugar.linear.x, self.posicion_pieza_jugar.linear.y, self.posicion_pieza_jugar.linear.z+0.05, self.posicion_pieza_jugar.angular.x, self.posicion_pieza_jugar.angular.y, self.posicion_pieza_jugar.angular.z]
            
            self.posicion_pieza_jugar_mas_alto = self.create_twist(array_posicion_pieza_jugar_mas_alto)
            
            trayectoria_jugada_usar = (self.posicion_camara_tablero, self.posicion_intermedia , 'open', self.posicion_camara_piezas_robot, self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'close', self.posicion_up_piezas_robot, self.posicion_intermedia, self.posicion_pieza_jugar_mas_alto , self.posicion_pieza_jugar, 'open' ,self.posicion_pieza_jugar_mas_alto,  self.posicion_camara_tablero)
            
            
            return trayectoria_jugada_usar
            
            
        else: #robar
            rospy.loginfo(f"{self.node_name }: No tengo pieza valida, robo")
            #print("No tengo pieza valida, robo")
            
            self.posicion_pieza_robot = self.elegir_posicion_pieza_robot() #nuevo aqui
            
            self.tipo_jugada = "robar_pieza"
            
            self.posicion_pieza_robar = self.create_twist([0.264, 0.346, 0.173, 3.089, -0.015, -1.214])
            
            array_posicion_pieza_robar_mas_alto=[self.posicion_pieza_robar.linear.x, self.posicion_pieza_robar.linear.y, self.posicion_pieza_robar.linear.z+0.05, self.posicion_pieza_robar.angular.x, self.posicion_pieza_robar.angular.y, self.posicion_pieza_robar.angular.z]
                
            self.posicion_pieza_robar_mas_alto = self.create_twist(array_posicion_pieza_robar_mas_alto)
                
            trayectoria_jugada_robar = (self.posicion_camara_tablero, 'open',  self.posicion_pieza_robar_mas_alto , self.posicion_pieza_robar, 'close', self.posicion_pieza_robar_mas_alto, self.posicion_cuna0, self.posicion_cuna1, self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_cuna5, self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'open', self.posicion_up_piezas_robot,self.posicion_camara_piezas_robot) #, self.posicion_intermedia,self.posicion_camara_tablero)
            
            
            #trayectoria_jugada_robar = ('open', self.posicion_camara_tablero_robar,'open')

            return trayectoria_jugada_robar
            



    # cuando roba evalúa en que posición dejarla
    def elegir_posicion_pieza_robot(self):
     
        self.iterador_pieza_nueva=0
        
        self.flag_elegir_posicion=False
        for i in range(len(self.valores_piezas)):
            if(self.valores_piezas[i][0] == False):  #es False, es decir, ahi no hay pieza, se puede colocar ahi:
            
                self.valores_piezas[i][0]= True
                self.valores_piezas[i][1]= 9 #random.randint(0,6)
                self.valores_piezas[i][2]= 9 #random.randint(0,6)
                self.flag_elegir_posicion= True
                #self.contador_piezas_robot = self.contador_piezas_robot + 1
                break	
                    
        
        self.contador_piezas_robot = self.contador_piezas_robot +1
        
        
        if(self.flag_elegir_posicion == True):
            self.iterador_pieza_nueva=i
            rospy.loginfo(f"{self.node_name }: Pongo la ficha en la posicion {i+1}")
            #print("Pongo la ficha en la posicion " + str(i+1) + " "  + str(self.valores_piezas))
            #print(self.valores_piezas)
            return self.posiciones_piezas_robot[i]
            
        else:
            self.iterador_pieza_nueva = i+1
            self.valores_piezas.append([True,9,9])
            #print(self.valores_piezas)
            return self.posiciones_piezas_robot[self.iterador_pieza_nueva]                    
                     
                     
                     
                     
    def create_twist(self,valores):
        new_Twist = Twist()
        new_Twist.linear.x=valores[0]
        new_Twist.linear.y=valores[1]
        new_Twist.linear.z=valores[2]
       
        new_Twist.angular.x=valores[3]
        new_Twist.angular.y=valores[4]
        new_Twist.angular.z=valores[5]
        
        return new_Twist
                             
if __name__ == '__main__':
    
    rospy.init_node('jugada') 
    obj=jugada()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









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

global turno
turno = None


class jugada:
    def __init__(self):

        global contador
        global finish_jugada

        #self.posiciones_piezas_robot = [[1.133, -0.843, 1.166, -1.876, -1.543, -2.354],[1.104, -1.057, 1.553, -2.071, -1.595, -2.466], [1.027, -1.236, 1.789, -2.061, -1.602, -2.513],[0.902, -1.458, 2.093, -2.155, -1.567, -2.622],[1.311, -0.837, 1.154, -1.879, -1.619, -2.195],[1.296, -1.052, 1.544, -2.053, -1.617, -2.205],[1.289, -1.268, 1.885, -2.189, -1.679, -2.236] ,   [1.439, -0.837, 1.153, -1.875, -1.599, -2.104], [1.403, -1.021, 1.479, -2.003, -1.579, -2.124],[1.395, -1.229, 1.817, -2.152, -1.59, -2.181]] #   con mas posiciones donde poner para robar
        
        self.posiciones_piezas_robot = [[1.226, -0.63, 0.779, -1.719, -1.634, -2.349], [1.165, -0.884, 1.261, -1.958, -1.595, -2.35],[1.132, -1.062, 1.555, -2.039, -1.624, -2.376], [1.376, -0.662, 0.857, -1.788, -1.657, -2.163], [1.337, -0.86, 1.205, -1.887, -1.631, -2.214], [1.302, -1.081, 1.593, -2.06, -1.634, -2.222], [1.479, -0.557, 0.616, -1.594, -1.609, -2.088], [1.476, -0.853, 1.205, -1.915, -1.631, -2.069],[1.458, -1.054, 1.535, -2.032, -1.631, -2.069]]

        #self.valores_piezas=[[True,3,5],[True,1,5],[True,0,2],[True,3,3],[True,6,4],[True,3,0],[True,0,6]]
        self.valores_piezas = [[False, 0, 0], [False, 0, 0], [False, 0, 0],[False, 0, 0], [False, 0, 0], [False, 0, 0], [False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0]]
        self.valores_piezas = [[True, 0, 0], [True, 0, 0], [True, 0, 0],[True, 0, 0], [False, 0, 0], [True, 0, 0], [False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0],[False, 0, 0]]
        
        #print("Mis piezas: " + str(self.valores_piezas))


        #publishers
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        self.publisher_articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=1)
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)
        
        #subscribers
        
        self.subscriber_finish_go_to_pose = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)
        #self.subscriber_init = rospy.Publisher('/play_robot/init', String, self.finish_collect_callback,  queue_size=10)
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1) 
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        self.subscriber_posicion_piezas_vision = rospy.Subscriber('/vision/posicion_piezas', String, self.posicion_piezas_callback, queue_size=1)
        #self.subscriber_valores_nuevas_piezas_robot = rospy.Subscriber('/vision/valores_nuevas_piezas', Float64MultiArray, self.valores_nuevas_piezas_callback, queue_size=1)
        
        #para obtener los valores de las piezas que se recogen
        self.publisher_valores_nuevas_piezas_robot = rospy.Subscriber('/vision/valores_nuevas_piezas_robot', Float64MultiArray, self.obtain_new_value_piece ,  queue_size=1)
        self.publisher_valores_todas_piezas_robot = rospy.Subscriber('/vision/valores_todas_piezas_robot', Float64MultiArray, self.obtain_value_all_pieces,  queue_size=1)
        	

        #inicializacion de variables de posicion
        self.send_pose = Twist()
        self.posicion_pieza = Twist()
        

        self.posicion_camara_tablero= [0.221, -1.144, 0.165, -0.594, -1.565, 4.555] #[0.094, -1.182, 0.514, -0.959, -1.578, -0.324] 
        self.posicion_camara_piezas_robot= [1.265, -0.998, 0.433, -0.946, -1.6, -2.253] #[1.377, -1.102, 0.48, -0.917, -1.576, -0.544] 
        self.posicion_up_piezas_robot= [1.388, -0.906, 0.414, -1.089, -1.602, -0.553]
        self.posicion_intermedia= [0.869, -0.98, 0.514, -1.255, -1.593, -0.562]

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
        
        
        
    #obtener los valores de las piezas que se roban
    def obtain_new_value_piece(self, data):
        #valores = Float64MultiArray()
        valores=data.data
        contador=0
        print(valores)
        #actualizar la pieza nueva
        
        for i in range(len(self.valores_piezas)):
            if(i==self.iterador_pieza_nueva ):
                break
            elif(self.valores_piezas[i][0]):
                contador=contador+1
        #probar si es verdad esto   
        self.valores_piezas[self.iterador_pieza_nueva][0] = True #valores[contador*3]
        self.valores_piezas[self.iterador_pieza_nueva][1] = valores[contador*3+1]
        self.valores_piezas[self.iterador_pieza_nueva][2] = valores[contador*3+2]
        
        
            
        print("Los valores de las piezas tras robar: " + str(self.valores_piezas))
        self.publisher_turn.publish("jugador")
        
        
        self.trayectoria_jugada = ("open", self.posicion_intermedia, self.posicion_camara_tablero)

        #publicar para que vaya a la posicion de vision del tablero              
        self.publisher_finish_go_to_pose.publish(True)
        
        
        #print("nuevos valores piezas: " + str(self.valores_piezas[self.iterador_pieza_nueva]))
        #print("Mis piezas: " + str(self.valores_piezas))
        #self.publisher_turn.publish("jugador")
        
        
        
        
        '''self.valores_piezas[self.iterador_pieza_nueva][0] = True
        self.valores_piezas[self.iterador_pieza_nueva][1]= data.data[0]
        self.valores_piezas[self.iterador_pieza_nueva][2]= data.data[1]
        print("nuevos valores piezas: " + str(self.valores_piezas[self.iterador_pieza_nueva]))
        print("Mis piezas: " + str(self.valores_piezas))
        self.publisher_turn.publish("jugador")'''
        
        #obtener los valores de todas las piezas que se han recogido
        
    #cuando recoge las 7:   
    def obtain_value_all_pieces(self, data):
        valores=data.data
        for i in range(len(valores)/3):
            if (valores[i*3] == 1):
                self.valores_piezas[i][0] = True
            else:   
                self.valores_piezas[i][0] = False
            self.valores_piezas[i][1] = valores[i*3+1]
            self.valores_piezas[i][2] = valores[i*3+2]
            
        print("Los valores de las piezas recogidas: " + str(self.valores_piezas))
        self.trayectoria_jugada = ("open", self.posicion_intermedia, self.posicion_camara_tablero)
        self.contador_piezas_robot = len(self.valores_piezas)
        #publicar para que vaya a la posicion de vision del tableto              
        self.publisher_finish_go_to_pose.publish(True)
        self.tipo_jugada = "colocar_camara"
        #self.publisher_turn.publish("jugador")
        

            
    def turn_callback(self, data):
        global turno
        turno = data.data
        if(data.data == "robot" or data.data == "ROBOT"):
        
            if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
        
                #posiciones donde vans la piezas del robot
                
                
                        
                #ya no tiene un pieza a la vista
                self.pieza_detectada = False
                
                self.trayectoria_jugada = self.elegir_pieza()
                
                self.publisher_finish_go_to_pose.publish(True)
                    
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
                print("Falta detectar pieza")





    
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
                                #print("open")
                            else:
                                self.publisher_gripper.publish("close")
                                #print("close")
                                
                        elif("Twist" in str(type(self.trayectoria_jugada[contador]))): #enviar la pose a la que tiene que ir el robot
                            self.send_pose=self.trayectoria_jugada[contador]
                            #print("pose")
                            
                            self.publisher_pose.publish(self.send_pose)
                            
                        else: #enviar la posicion articular a la que tiene que ir el robot
                            #print("Articular")
                            self.send_articular.data= self.trayectoria_jugada[contador]
                            self.publisher_articular.publish(self.send_articular)           
                            

                        contador = contador +1
                        
            else:
                #comprobar si el robot ha ganado:
                if(self.contador_piezas_robot == 0):
                    print("El robot ha ganado")
                    self.publisher_turn.publish("victoria_robot")
                    
                    
                    
                else:
                    print("Me quedan " + str(self.contador_piezas_robot) + " fichas")
                    
                    if(self.tipo_jugada == "robar"): # se ha robado
                        
                        self.publisher_turn.publish("evaluando")
                        
                        #llamar a vision para reconocer las piezas nuevas
                        '''self.valores_piezas[self.iterador_pieza_nueva][1]= random.randint(0,6)
                        self.valores_piezas[self.iterador_pieza_nueva][2]= random.randint(0,6)
                        print("nuevos valores piezas: " + str(self.valores_piezas[self.iterador_pieza_nueva]))'''
                        
                        # print("Mis piezas: " + str(self.valores_piezas))
                        
                    else: # se ha jugado
                    
                        self.publisher_turn.publish("jugador")
                        print("Mis piezas: " + str(self.valores_piezas))
                    
                    #self.publisher_jugada.publish(True)
                contador=0

        else:  
            contador=0
        
    def posicion_piezas_callback(self, data):
	
        #print(data)
        
        #poner las posiciones obtenidas de la vision
                
        self.posicion_pieza_extremo = self.create_twist([0.364, 0.073, 0.177, -3.141, -0.06, -1.161])
        
        self.posicion_pieza_robar = self.create_twist([0.264, 0.346, 0.173, 3.089, -0.015, -1.214])
        
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
        self.flag = False
        self.tipo_jugada = ""
        for i in range(len(self.valores_piezas)):
            if(self.valores_piezas[i][0]):  #es true, es decir, aun no se ha usado:
                if(self.valores_piezas[i][1] == valor_extremo or self.valores_piezas[i][2]  == valor_extremo ):
                    self.valores_piezas[i][0]= False
                    self.flag= True
                    self.contador_piezas_robot = self.contador_piezas_robot -1
                    break
                    
        
        
        if(self.flag == True):
            self.tipo_jugada = "jugar"
            print("Cojo la ficha " + str(i+1))# + " "  + str(self.valores_piezas))
            
            self.posicion_pieza_robot = self.posiciones_piezas_robot[i]
            array_posicion_pieza_extremo_mas_alto=[self.posicion_pieza_extremo.linear.x, self.posicion_pieza_extremo.linear.y, self.posicion_pieza_extremo.linear.z+0.05, self.posicion_pieza_extremo.angular.x, self.posicion_pieza_extremo.angular.y, self.posicion_pieza_extremo.angular.z]
            
            self.posicion_pieza_extremo_mas_alto = self.create_twist(array_posicion_pieza_extremo_mas_alto)
            
            trayectoria_jugada_usar = (self.posicion_camara_tablero, self.posicion_intermedia , 'open', self.posicion_camara_piezas_robot, self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'close', self.posicion_up_piezas_robot, self.posicion_intermedia,  self.posicion_camara_tablero , self.posicion_pieza_extremo_mas_alto , self.posicion_pieza_extremo, 'open' ,self.posicion_pieza_extremo_mas_alto,  self.posicion_camara_tablero)
            
            
            return trayectoria_jugada_usar
            
            
        else: #robar
            print("No tengo pieza valida, robo")
            self.tipo_jugada = "robar"
            self.posicion_pieza_robot = self.elegir_posicion_pieza()
            
            array_posicion_pieza_robar_mas_alto=[self.posicion_pieza_robar.linear.x, self.posicion_pieza_robar.linear.y, self.posicion_pieza_robar.linear.z+0.05, self.posicion_pieza_robar.angular.x, self.posicion_pieza_robar.angular.y, self.posicion_pieza_robar.angular.z]
                
            self.posicion_pieza_robar_mas_alto = self.create_twist(array_posicion_pieza_robar_mas_alto)
                
            trayectoria_jugada_robar = (self.posicion_camara_tablero, 'open',  self.posicion_pieza_robar_mas_alto , self.posicion_pieza_robar, 'close', self.posicion_pieza_robar_mas_alto, self.posicion_cuna0, self.posicion_cuna1, self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_cuna5, self.posicion_up_piezas_robot, self.posicion_pieza_robot, 'open', self.posicion_up_piezas_robot,self.posicion_camara_piezas_robot) #, self.posicion_intermedia,self.posicion_camara_tablero)
            
            return trayectoria_jugada_robar
            



        
    def elegir_posicion_pieza(self):
     
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
            print("Pongo la ficha en la posicion " + str(i+1) + " "  + str(self.valores_piezas))
            #print(self.valores_piezas)
            return self.posiciones_piezas_robot[i]
            
        else:
            self.iterador_pieza_nueva = i+1
            self.valores_piezas.append([True,9,9])
            #print(self.valores_piezas)
            return self.posiciones_piezas_robot[self.iterador_pieza_nueva]                    
                     
                     
                     
if __name__ == '__main__':
    
    rospy.init_node('jugada') 
    obj=jugada()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









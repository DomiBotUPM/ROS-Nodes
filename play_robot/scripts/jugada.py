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

global contador
contador = 0
global msg

class jugada:
    def __init__(self):

        global contador
        global finish_jugada
        self.posiciones_piezas_robot=([0,0,0,0,0,0],[1,1,1,1,1,1],[2,2,2,2,2,2],[3,3,3,3,3,3],[4,4,4,4,4,4],[5,5,5,5,5,5])
        self.valores_piezas=[[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0]]
        #turno=""
    # conjuntos de acciones
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],"open",[0.3,0.3,0.3,-3.1,-0.1,0],"close","open","close",[0.3,0.2,0.3,-3.1,-0.1,0],"open","close" ,[0.3,0.2,0.3,-3.14,0,1.6])
        
	    #self.poses=jugada_poses #([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14])
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.25,0.38,-3.2,0.5,0.3])

	#buena posicion [0.3,0.0,0.3,-3.14,0,1.9]
        
        #movimientos:
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)
        
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


    def turn_callback(self, data):


        global turno
        turno= data.data
        
        #hacer la trayectoria aqui y la evaluacion llamando a elegir_pieza()
        if(self.pieza_detectada):
            elegir_pieza()
        else:
            print("Falta la vision de la pieza")
        
        
        self.trayectoria_jugada = ("open", self.posicion_pieza_elegida, "close", self.posicion_extremo_elegido,"open" ,self.posicion_camara)
            
    def elegir_pieza(self):
        #falta orientacion para ponerlo alineado segun las piezas
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
                    print("Esta pieza no vale")
                
                
    
    def jugada_callback(self, data):


        global finish_jugada
        finish_jugada= data.data
        

    def pose_callback(self, data):

        global contador
        global finish_jugada
        global turno

        if(turno == "robot" or turno == "ROBOT"):
            #if(finish_jugada):
            if(contador< len(self.trayectoria_jugada)):
                if (data.data == True):
                        #print (contador)
                            
                        if(isinstance(self.trayectoria_jugada[contador], str)): #si es open o close
                            if(str(self.trayectoria_jugada[contador]) == "open"):
                                self.publisher_gripper.publish("open")
                                print("open")
                            else:
                                self.publisher_gripper.publish("close")
                                print("close")
                        else: #enviar la pose a la que tiene que ir el robot           
                            '''self.send_pose.linear.x=self.trayectoria_jugada[contador][0]
                            self.send_pose.linear.y=self.trayectoria_jugada[contador][1]
                            self.send_pose.linear.z=self.trayectoria_jugada[contador][2]
                           
                            self.send_pose.angular.x=self.trayectoria_jugada[contador][3]
                            self.send_pose.angular.y=self.poses[contador][4]
                            self.send_pose.angular.z=self.poses[contador][5]'''
                            self.send_pose=self.trayectoria_jugada[contador]
                            print(self.send_pose)
                            self.publisher_pose.publish(self.send_pose)

                        contador = contador +1
                else:

                    self.publisher_turn.publish("jugador")
                    self.publisher_jugada.publish(True)
                    contador=0
                    #time.sleep(500)

                    #rospy.signal_shutdown("Shutting Down")

            #else:
               #contador=0
        else:  
            contador=0
        
    def posicion_piezas_callback(self, data):
	
        print(data)
        #poner las posiciones obtenidas de la vision
        
        
        self.posicion_pieza_extremo1.linear.x= 0.2 #self.poses[contador][0]
        self.posicion_pieza_extremo1.linear.y= 0.2 #self.poses[contador][1]
        self.posicion_pieza_extremo1.linear.z= 0.2 #self.poses[contador][2]
       
        self.posicion_pieza_extremo1.angular.x= -3.14 #self.poses[contador][3]
        self.posicion_pieza_extremo1.angular.y= 0.0 #self.poses[contador][4]
        self.posicion_pieza_extremo1.angular.z= 0.3 #self.poses[contador][5]
        
        self.posicion_pieza_extremo2.linear.x= 0.2 #self.poses[contador][0]
        self.posicion_pieza_extremo2.linear.y= 0.2 #self.poses[contador][1]
        self.posicion_pieza_extremo2.linear.z= 0.2 #self.poses[contador][2]
       
        self.posicion_pieza_extremo2.angular.x= -3.14 #self.poses[contador][3]
        self.posicion_pieza_extremo2.angular.y= 0.0 #self.poses[contador][4]
        self.posicion_pieza_extremo2.angular.z= 0.3 #self.poses[contador][5]
        
        self.valor_izquierda_arriba_extremo1=1
        self.valor_derecha_abajo_extremo1=1
        
        self.valor_izquierda_arriba_extremo2=1
        self.valor_derecha_abajo_extremo2=1
        
        #print(str(self.posicion_pieza))
        self.pieza_detectada = True
        
        
if __name__ == '__main__':
    #global msg
    rospy.init_node('poses_and_gripper') #_node
    #jugada_poses  = ([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14])
    obj=jugada()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









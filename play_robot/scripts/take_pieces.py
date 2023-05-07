#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import time

global contador_piezas
contador_piezas = 0

global contador_movimientos
contador_movimientos = 0

global numpiezas
numpiezas=7
global msg


class pieces:
    def __init__(self):

        global contador_piezas
        global numpiezas
        
	self.posiciones_piezas_robot=([0,0,0,0,0,0],[1,1,1,1,1,1],[2,2,2,2,2,2],[3,3,3,3,3,3],[4,4,4,4,4,4],[5,5,5,5,5,5])
	#([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9],[0.3,0.2,0.3,-3.14,0,-2.9],[0.3,0.2,0.3,-3.14,0,-2.9])
        #self.posicion_camara= [8,8,8,8,8,8]
           	
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1)
        
        self.publisher_init = rospy.Publisher('/play_robot/init', String, queue_size=10)
        
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1)  
        
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
        
        self.subscriber_posicion_piezas_vision = rospy.Subscriber('/vision/posicion_piezas', String, self.posicion_piezas_callback, queue_size=1)

        #para publicar los movimientos:
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
        
        self.publisher_finish = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=10)
        
        #inicialicion de variables
        self.posicion_pieza_robot = Twist()
        self.posicion_pieza = Twist()
        self.send_pose = Twist()
        self.init= String()
        self.pieza_detectada = Bool()
        
        #prueba
        self.posicion_camara= Twist()
        self.posicion_camara.linear.x=8
        self.posicion_camara.linear.y=8
        self.posicion_camara.linear.z=8

        self.posicion_camara.angular.x=8
        self.posicion_camara.angular.y=8
        self.posicion_camara.angular.z=8

        
        
    def init_callback(self, data):
        self.init=data.data
        if(self.init == "finish" or self.init == "FINISH"):
            print("Finalizada la recogida de piezas")

    def jugada_callback(self, data): #crea la trayectoria
    
        global contador_piezas
        global numpiezas
        if (self.init == 'init' or self.init == 'INIT'):
            if(data.data == True):
                if(contador_piezas < numpiezas):
                    if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
                
                        print(str(self.posicion_pieza))
                        '''Llamar a un poses_and_gripper actualizado con las coordenadas de la vision  


                        posicion predeterminada para las piezas del robot'''
                        self.posicion_pieza_robot.linear.x=self.posiciones_piezas_robot[contador_piezas][0]
                        self.posicion_pieza_robot.linear.y=self.posiciones_piezas_robot[contador_piezas][1]
                        self.posicion_pieza_robot.linear.z=self.posiciones_piezas_robot[contador_piezas][2]

                        self.posicion_pieza_robot.angular.x=self.posiciones_piezas_robot[contador_piezas][3]
                        self.posicion_pieza_robot.angular.y=self.posiciones_piezas_robot[contador_piezas][4]
                        self.posicion_pieza_robot.angular.z=self.posiciones_piezas_robot[contador_piezas][5]

                        print(str(self.posicion_pieza_robot))


                        #self.trayectoria_jugada = ("open", [self.posicion_pieza.linear.x, self.posicion_pieza.linear.y,self.posicion_pieza.linear.z, self.posicion_pieza.angular.x, self.posicion_pieza.angular.y, self.posicion_pieza.angular.z], "close", [self.posicion_pieza_robot.linear.x, self.posicion_pieza_robot.linear.y,self.posicion_pieza_robot.linear.z, self.posicion_pieza_robot.angular.x, self.posicion_pieza_robot.angular.y, self.posicion_pieza_robot.angular.z],"open" ,self.posicion_camara)
                        
                        self.trayectoria_jugada = ("open", self.posicion_pieza, "close", self.posicion_pieza_robot,"open" ,self.posicion_camara)

                        self.pieza_detectada = False
                        print(str(self.trayectoria_jugada))
                        self.publisher_finish.publish(True)

                        #time.sleep(1)
                        contador_piezas= contador_piezas+1
                    else:
                        print("Falta detectar pieza")
                       
                       
                       
                else:
                    self.publisher_init.publish("finish")
                    contador=0
        else:
            print("Finalizada la recogida de la pieza")
        #time.sleep(500)

        #rospy.signal_shutdown("Shutting Down")

    def posicion_piezas_callback(self, data):
        print(data)
        #poner las posiciones obtenidas de la vision
        
        
        self.posicion_pieza.linear.x= 0.2 #self.poses[contador][0]
        self.posicion_pieza.linear.y= 0.2 #self.poses[contador][1]
        self.posicion_pieza.linear.z= 0.2 #self.poses[contador][2]
       
        self.posicion_pieza.angular.x= -3.14 #self.poses[contador][3]
        self.posicion_pieza.angular.y= 0.0 #self.poses[contador][4]
        self.posicion_pieza.angular.z= 0.3 #self.poses[contador][5]
        #print(str(self.posicion_pieza))
        self.pieza_detectada = True
	
	
    def pose_callback(self, data):
	
        global contador_movimientos
      

        #print(turno)
        #print("El valor es " + str(data.data == 1) + " con data = " +str(data.data))

        if(contador_movimientos< len(self.trayectoria_jugada)):
            if (data.data == True):
                    #print (contador)
                        
                    if(isinstance(self.trayectoria_jugada[contador_movimientos], str)): #si es open o close
                        if(str(self.trayectoria_jugada[contador_movimientos]) == "open"):
                            self.publisher_gripper.publish("open")
                            print("open")
                        else:
                            self.publisher_gripper.publish("close")
                            print("close")
                    else: #enviar la pose a la que tiene que ir el robot           
                        '''
                        self.send_pose.linear.x=self.trayectoria_jugada[contador_movimientos][0]
                        self.send_pose.linear.y=self.trayectoria_jugada[contador_movimientos][1]
                        self.send_pose.linear.z=self.trayectoria_jugada[contador_movimientos][2]
                       
                        self.send_pose.angular.x=self.trayectoria_jugada[contador_movimientos][3]
                        self.send_pose.angular.y=self.trayectoria_jugada[contador_movimientos][4]
                        self.send_pose.angular.z=self.trayectoria_jugada[contador_movimientos][5]'''
                        self.send_pose=self.trayectoria_jugada[contador_movimientos]
                        print(self.send_pose)
                        self.publisher_pose.publish(self.send_pose)

                    contador_movimientos = contador_movimientos +1
            else:
                    rospy.loginfo(rospy.get_caller_id() + " Esperando")
        else:
            #self.publisher_finish.publish(False)
            self.publisher_jugada.publish(True)
            contador_movimientos=0
            print("jugada realizada")
            #time.sleep(500)

            #rospy.signal_shutdown("Shutting Down")
	
            
            
if __name__ == '__main__':
    #global msg
    rospy.init_node('take_pieces') #_node
    obj=pieces()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









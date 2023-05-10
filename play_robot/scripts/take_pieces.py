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
from std_msgs.msg import Float64MultiArray
from moveit_commander.conversions import pose_to_list
import time
import random

global contador_piezas
contador_piezas = 0

global contador_movimientos
contador_movimientos = 0

global numpiezas
numpiezas=7
global msg

#global valores_piezas
#valores_piezas=[[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0]]

class pieces:
    def __init__(self):

        global contador_piezas
        global numpiezas
        self.valores_piezas=[[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0],[False,0,0]]#cambio aqui
        self.valores_piezas_robot = [False, 0, 0, 0, 0, 0, True, 0, 0,True, 0, 0, True, 0, 0, True, 0, 0, True, 0, 0]

        
        #self.posiciones_piezas_robot= ([0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[-0.006, 0.425, 0.175, -3.14, 0.047, 2.079],[0.012, 0.459, 0.171, -3.086, -0.027, 2.029],[0.009, 0.4, 0.172, -3.12, 0.011, 1.999],[-0.048, 0.517, 0.172, 3.103, -0.007, 2.015])
        self.posiciones_piezas_robot= ([0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[-0.036, 0.53, 0.176, 3.122, 0.01, -1.105],[-0.012, 0.531, 0.177, -3.123, -0.009, 1.969],[0.009, 0.4, 0.172, -3.12, 0.011, 1.999],[-0.048, 0.517, 0.172, 3.103, -0.007, 2.015])
           	
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1)
        
        self.publisher_init = rospy.Publisher('/play_robot/init', String, queue_size=10)
        
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1)  
        
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
        
        self.subscriber_posicion_piezas_vision = rospy.Subscriber('/vision/posicion_piezas', String, self.posicion_piezas_callback, queue_size=1)

        #para publicar los movimientos:
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
        
        #self.publisher_finish = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=10)
        
        self.publisher_valores_piezas = rospy.Publisher('/posiciones_piezas_robot', Float64MultiArray, queue_size=10)
        
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        
                
        #inicialicion de variables
        self.posicion_pieza_robot = Twist()
        self.posicion_pieza = Twist()
        self.send_pose = Twist()
        self.init= String()
        self.pieza_detectada = Bool()
        
        self.posicion_pieza_robot_mas_alto= Twist()
        self.posicion_pieza_mas_alto= Twist()
        
        self.valores_piezas_robot_send = Float64MultiArray()
        #self.valores_piezas_robot_send.data= self.valores_piezas_robot 
        
        self.posicion_cuna0 = Twist()
        self.posicion_cuna1 = Twist() # [0.154, 0.333, 0.195, -3.126, 0.021, -2.75]
        self.posicion_cuna2 = Twist() # [0.153, 0.367, 0.189, 3.093, 0.05, -2.774]
        self.posicion_cuna3 = Twist() # [0.153, 0.358, 0.227, 3.022, 0.082, -2.779]
        self.posicion_cuna4 = Twist() # [0.148, 0.408, 0.221, 2.961, 0.107, -2.77]
        self.posicion_cuna5 = Twist()
        
        self.posicion_up_robot=self.create_twist([0.0, 0.224, 0.694, -1.574, 0.0, -0.001])
        self.posicion_up_piezas_robot=self.create_twist([-0.027, 0.474, 0.314, -3.13, 0.014, 1.979]) 
        self.posicion_intermedia=self.create_twist([0.106, 0.382, 0.35, -3.063, -0.035, 0.021])

        #prueba
	#[-0.0904,0.41,0.388,3.1,-0.07,0.3625]
	#[0.3283492383502373, 0.11622792170424899, 0.4198220432735271, -2.782066102328459, -0.23737192306442884, -1.2553678309409804]

        self.posicion_camara= self.create_twist([0.307, 0.192, 0.372, -3.119, -0.022, -1.215])
        
        #cunas
        
         
        
        #cuna0 # [0.154, 0.333, 0.295, -3.126, 0.021, -2.75]
        self.posicion_cuna0 = self.create_twist([0.153, 0.334, 0.295, -3.141, -0.019, 0.404])
        
        #cuna1 # [0.154, 0.333, 0.195, -3.126, 0.021, -2.75]
        self.posicion_cuna1 = self.create_twist([0.153, 0.334, 0.195, -3.141, -0.019, 0.404])
        
        ##cuna2 [0.153, 0.367, 0.189, 3.093, 0.05, -2.774]
        self.posicion_cuna2 = self.create_twist([0.153, 0.367, 0.189, -3.141, -0.019, 0.404])      
        
        #cuna3  [0.153, 0.358, 0.227, 3.022, 0.082, -2.779]
        self.posicion_cuna3 = self.create_twist([0.153, 0.358, 0.23, -3.141, -0.019, 0.404])
        
        #cuna 4 [0.148, 0.408, 0.221, 2.961, 0.107, -2.77]
        self.posicion_cuna4 = self.create_twist([0.148, 0.408, 0.23, -3.141, -0.019, 0.404])
        
        #cuna 5 [0.148, 0.408, 0.221, 2.961, 0.107, -2.77]
        self.posicion_cuna5 = self.create_twist([0.148, 0.408, 0.4, -3.141, -0.019, 0.404])
        
        
    def init_callback(self, data):
        self.init=data.data
        if(self.init == "init" or self.init == "init"):
        #para que empiece el movimiento:
            self.publisher_jugada.publish(True)
        if(self.init == "finish" or self.init == "FINISH"):
            print("Finalizada la recogida de piezas")
            
    def jugada_callback(self, data): #crea la trayectoria
    
        global contador_piezas
        global numpiezas
        if (self.init == 'init' or self.init == 'INIT'):
            if(data.data == True):
                if(contador_piezas < numpiezas):
                    if(self.pieza_detectada == True): ## Tener cuidado aqui, controlar bien la publicacion de la posicion de la vision y luego empezar la jugada
                
                        #posiciones donde vans la piezas del robot
                        array_posicion_piezas_robot=[self.posiciones_piezas_robot[contador_piezas][0],self.posiciones_piezas_robot[contador_piezas][1],self.posiciones_piezas_robot[contador_piezas][2],self.posiciones_piezas_robot[contador_piezas][3],self.posiciones_piezas_robot[contador_piezas][4],self.posiciones_piezas_robot[contador_piezas][5]]
                        
                        self.posicion_pieza_robot = self.create_twist(array_posicion_piezas_robot)
                        
      
                        #posiciones de seguridad
                        array_posicion_pieza_robot_mas_alto=[self.posicion_pieza_robot.linear.x, self.posicion_pieza_robot.linear.y, self.posicion_pieza_robot.linear.z +0.05, self.posicion_pieza_robot.angular.x, self.posicion_pieza_robot.angular.y, self.posicion_pieza_robot.angular.z]
                        self.posicion_pieza_robot_mas_alto = self.create_twist(array_posicion_pieza_robot_mas_alto)
                        
                        array_posicion_pieza_mas_alto=[self.posicion_pieza.linear.x, self.posicion_pieza.linear.y, self.posicion_pieza.linear.z+0.05, self.posicion_pieza.angular.x, self.posicion_pieza.angular.y, self.posicion_pieza.angular.z]
                        self.posicion_pieza_mas_alto = self.create_twist(array_posicion_pieza_mas_alto)
   
                        
                        #creacion de trayectoria

                        self.trayectoria_jugada = ("open",self.posicion_camara, self.posicion_pieza_mas_alto, self.posicion_pieza, "close",self.posicion_pieza_mas_alto,self.posicion_cuna0, self.posicion_cuna1, self.posicion_cuna2, self.posicion_cuna3, self.posicion_cuna4, self.posicion_up_piezas_robot  ,self.posicion_pieza_robot_mas_alto, self.posicion_pieza_robot, "open", self.posicion_pieza_robot_mas_alto, self.posicion_up_piezas_robot, self.posicion_intermedia, self.posicion_camara) # self.posicion_camara, self.posicion_up_robot,
                        #ya no tiene un pieza a la vista
                        self.pieza_detectada = False

                        
                        #publicacion de las posiciones y valores del robot 
                        '''self.valores_piezas[contador_piezas]=[True,self.valor_izquierda_arriba,self.valor_derecha_abajo]
                        
                        #asignacion de los valores de las piezas - mejor pasarlo a jugada
                        self.valores_piezas_robot[contador_piezas*3] = True
                        self.valores_piezas_robot[contador_piezas*3+1] = self.valor_izquierda_arriba
                        self.valores_piezas_robot[contador_piezas*3+2] = self.valor_derecha_abajo
                        
                        self.valores_piezas_robot_send.data = self.valores_piezas_robot'''
                        
                        #publicar los valores de las piezas
                        
                        self.publisher_finish_go_to_pose.publish(True)
                        

                        contador_piezas= contador_piezas+1
                        
                        
                    else:
                        print("Falta detectar pieza")
                       
                       
                       
                else:
                    self.publisher_init.publish("finish")
                    #print("Mensaje a publicar:")
                    #print(self.valores_piezas_robot_send)
                    self.publisher_valores_piezas.publish(self.valores_piezas_robot_send)
                    contador=0
        else:
            print("Finalizada la recogida de la pieza")
            #print(str(self.valores_piezas))
            
            
            
        #time.sleep(500)

        #rospy.signal_shutdown("Shutting Down")

    def posicion_piezas_callback(self, data):
        print(data)
        #poner las posiciones obtenidas de la vision

	    #[0.25, 0.312, 0.173, -3.092, -0.027, 1.988]  [0.253, 0.31, 0.226, 3.118, -0.01, -1.252]

        self.posicion_pieza= self.create_twist([0.253, 0.31, 0.173, 3.118, -0.01, -1.252])
        
        #tomar valores para las piezas - pasarlo a jugada
        '''self.valor_izquierda_arriba= random.randint(0,6) #1 #cambio aqui
        self.valor_derecha_abajo=random.randint(0,6)    #cambio aqui'''

        #print(str(self.posicion_pieza))
        self.pieza_detectada = True
            
        self.publisher_jugada.publish(True)
	
    def pose_callback(self, data):
	
        global contador_movimientos
      

        #print(turno)
        #print("El valor es " + str(data.data == 1) + " con data = " +str(data.data))

        if(contador_movimientos< len(self.trayectoria_jugada)):
            if (data.data == True and self.trayectoria_jugada !=None): #cambio aqui
                    #print (contador)
                        
                    if(isinstance(self.trayectoria_jugada[contador_movimientos], str)): #si es open o close
                        if(str(self.trayectoria_jugada[contador_movimientos]) == "open"):
                            self.publisher_gripper.publish("open")
                            print("open")
                        else:
                            self.publisher_gripper.publish("close")
                            print("close")
                    else: #enviar la pose a la que tiene que ir el robot           

                        self.send_pose=self.trayectoria_jugada[contador_movimientos]

                        print("pose")
                        self.publisher_pose.publish(self.send_pose)

                    contador_movimientos = contador_movimientos +1
            else:
                    print("Esperando")
        else:
            #vaciado de la trayectoria
            self.trayectoria_jugada=None # cambio aqui
            
            self.publisher_jugada.publish(True)
            contador_movimientos=0
            #print("jugada realizada")
            print('Llevo ' + str(contador_piezas) + ' piezas')

	
            
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
    #global msg
    rospy.init_node('take_pieces') #_node
    obj=pieces()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









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
	#self.posiciones_piezas_robot=([0.34,-0.09,0.227,3.107,-0.05,-1.21],[0.34,0,0.227,3.107,-0.05,-1.21],[0.34,0.1,0.227,3.107,-0.05,-1.21],[0.34,0.2,0.227,3.107,-0.05,-1.21],[0.34,0.3,0.227,3.107,-0.05,-1.21],[0.34,-0.09,0.227,3.107,-0.05,-1.21],[0.34,0,0.227,3.107,-0.05,-1.21])

	#([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9],[0.3,0.2,0.3,-3.14,0,-2.9],[0.3,0.2,0.3,-3.14,0,-2.9])
        #self.posicion_camara= [8,8,8,8,8,8]
        
        self.posiciones_piezas_robot= ([0.238, 0.187, 0.315, -3.121, -0.0256, -1.156],[0.2498, 0.146, 0.30845, 3.129, -0.0652, -1.122],[0.270, 0.092, 0.3037, 3.077, -0.0398, -1.1618],[0.2806, 0.0457, 0.2997, 3.0508, -0.0657, -1.177],[0.2813, -0.0084, 0.2971, 3.0168, -0.02, -1.14],[0.2842, -0.06, 0.295, 3.014, -0.0425, -1.1166]
,[0.2879, -0.1125, 0.2797, 3.0146, -0.028, -1.091]) #([-0.202,0.3822,0.259,3.13,0.04,0.36],[-0.149,0.393,0.263,3.09,0.0293,0.326],[-0.1044,0.4,0.2666,3.06,-0.011,0.403],[-0.05,0.4,0.259,3.02,0.036,0.338],[0.0048,0.399,0.265,3.054,0.046,0.35],[0.044,0.398,0.257,3.02,0.003,0.3639],[0.0957,0.3927,0.2679,3.08,0.0088,0.37])
        
           	
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
        
        self.publisher_valores_piezas = rospy.Publisher('/posiciones_piezas_robot', Float64MultiArray, queue_size=10)
        
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


        #prueba
	#[-0.0904,0.41,0.388,3.1,-0.07,0.3625]
	#[0.3283492383502373, 0.11622792170424899, 0.4198220432735271, -2.782066102328459, -0.23737192306442884, -1.2553678309409804]

        self.posicion_camara= Twist()
        '''self.posicion_camara.linear.x=-0.0904
        self.posicion_camara.linear.y=0.41
        self.posicion_camara.linear.z=0.388

        self.posicion_camara.angular.x=3.1
        self.posicion_camara.angular.y=-0.07
        self.posicion_camara.angular.z=0.3625'''
        
        self.posicion_camara.linear.x= 0.3283
        self.posicion_camara.linear.y= 0.1162
        self.posicion_camara.linear.z= 0.4198
       
        self.posicion_camara.angular.x= -2.782
        self.posicion_camara.angular.y= -0.2373
        self.posicion_camara.angular.z= -1.2553 #-2.6
	

        
        
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
                
                        #print(str(self.posicion_pieza))
                        '''Llamar a un poses_and_gripper actualizado con las coordenadas de la vision  


                        posicion predeterminada para las piezas del robot'''
                        self.posicion_pieza_robot.linear.x=self.posiciones_piezas_robot[contador_piezas][0]
                        self.posicion_pieza_robot.linear.y=self.posiciones_piezas_robot[contador_piezas][1]
                        self.posicion_pieza_robot.linear.z=self.posiciones_piezas_robot[contador_piezas][2]

                        self.posicion_pieza_robot.angular.x=self.posiciones_piezas_robot[contador_piezas][3]
                        self.posicion_pieza_robot.angular.y=self.posiciones_piezas_robot[contador_piezas][4]
                        self.posicion_pieza_robot.angular.z=self.posiciones_piezas_robot[contador_piezas][5]
                        
                        #print(str(self.posicion_pieza_robot))


                        #self.trayectoria_jugada = ("open", [self.posicion_pieza.linear.x, self.posicion_pieza.linear.y,self.posicion_pieza.linear.z, self.posicion_pieza.angular.x, self.posicion_pieza.angular.y, self.posicion_pieza.angular.z], "close", [self.posicion_pieza_robot.linear.x, self.posicion_pieza_robot.linear.y,self.posicion_pieza_robot.linear.z, self.posicion_pieza_robot.angular.x, self.posicion_pieza_robot.angular.y, self.posicion_pieza_robot.angular.z],"open" ,self.posicion_camara)
                        
                        #posiciones de seguridad
                        self.posicion_pieza_robot_mas_alto.linear.x=self.posicion_pieza_robot.linear.x
                        self.posicion_pieza_robot_mas_alto.linear.y=self.posicion_pieza_robot.linear.y
                        self.posicion_pieza_robot_mas_alto.linear.z=self.posicion_pieza_robot.linear.z + 0.1

                        self.posicion_pieza_robot_mas_alto.angular.x=self.posicion_pieza_robot.angular.x
                        self.posicion_pieza_robot_mas_alto.angular.y=self.posicion_pieza_robot.angular.x
                        self.posicion_pieza_robot_mas_alto.angular.z=self.posicion_pieza_robot.angular.x
                        
                        self.posicion_pieza_mas_alto.linear.x=self.posicion_pieza.linear.x
                        self.posicion_pieza_mas_alto.linear.y=self.posicion_pieza.linear.y
                        self.posicion_pieza_mas_alto.linear.z=self.posicion_pieza.linear.z + 0.1

                        self.posicion_pieza_mas_alto.angular.x=self.posicion_pieza.angular.x
                        self.posicion_pieza_mas_alto.angular.y=self.posicion_pieza.angular.x
                        self.posicion_pieza_mas_alto.angular.z=self.posicion_pieza.angular.x
                        
                        #creacion de trayectoria
                        
                        #self.trayectoria_jugada = ("open", self.posicion_pieza_mas_alto, self.posicion_pieza,  "close", self.posicion_pieza_mas_alto, self.posicion_pieza_robot_mas_alto , self.posicion_pieza_robot, "open", self.posicion_pieza_robot_mas_alto  ,self.posicion_camara)
                        
                        self.trayectoria_jugada = ("open", self.posicion_pieza,  "close",  self.posicion_pieza_robot, "open" ,self.posicion_camara)

                        self.pieza_detectada = False
                        #print(str(self.trayectoria_jugada))
                        
                        #publicacion de las posiciones y valores del robot 
                        self.valores_piezas[contador_piezas]=[True,self.valor_izquierda_arriba,self.valor_derecha_abajo]
                        
                        #self.posiciones_piezas_robot[contador_piezas]=[True,self.valor_izquierda_arriba,self.valor_derecha_abajo]
                        
                        self.valores_piezas_robot[contador_piezas*3] = True
                        self.valores_piezas_robot[contador_piezas*3+1] = self.valor_izquierda_arriba
                        self.valores_piezas_robot[contador_piezas*3+2] = self.valor_derecha_abajo
                        
                        self.valores_piezas_robot_send.data = self.valores_piezas_robot
                        
                        #publicar los valores de las piezas
                        #self.publisher_valores_piezas.publish(self.valores_piezas_robot_send)
                        self.publisher_finish.publish(True)
                        #print(self.valores_piezas_robot_send.data)
                        #time.sleep(1)
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
        
        #[-0.173,0.3168,0.224,3.02,0.05,0.302]
        #[0.4407235856721363, 0.11290455432397502, 0.27139535780965884, -2.9428261157424873, -0.2069397855228445, -1.1727219124636508]


        self.posicion_pieza.linear.x= 0.4407 #self.poses[contador][0]
        self.posicion_pieza.linear.y= 0.1129 #self.poses[contador][1]
        self.posicion_pieza.linear.z= 0.2714 #self.poses[contador][2]
       
        self.posicion_pieza.angular.x= -2.9428 #self.poses[contador][3]
        self.posicion_pieza.angular.y= -0.2069 #self.poses[contador][4]
        self.posicion_pieza.angular.z= -1.1727 #-2.6 #self.poses[contador][5]
        
        #tomar valores para 
        self.valor_izquierda_arriba= random.randint(0,6) #1 #cambio aqui
        self.valor_derecha_abajo=random.randint(0,6)    #cambio aqui

        #print(str(self.posicion_pieza))
        self.pieza_detectada = True
	
	
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
                        '''
                        self.send_pose.linear.x=self.trayectoria_jugada[contador_movimientos][0]
                        self.send_pose.linear.y=self.trayectoria_jugada[contador_movimientos][1]
                        self.send_pose.linear.z=self.trayectoria_jugada[contador_movimientos][2]
                       
                        self.send_pose.angular.x=self.trayectoria_jugada[contador_movimientos][3]
                        self.send_pose.angular.y=self.trayectoria_jugada[contador_movimientos][4]
                        self.send_pose.angular.z=self.trayectoria_jugada[contador_movimientos][5]'''
                        self.send_pose=self.trayectoria_jugada[contador_movimientos]
                        #print(self.send_pose)
                        print("pose")
                        self.publisher_pose.publish(self.send_pose)

                    contador_movimientos = contador_movimientos +1
            else:
                    #rospy.loginfo(rospy.get_caller_id() + " Esperando")
                    print("Esperando")
        else:
            #self.publisher_finish.publish(False)
            self.trayectoria_jugada=None#cambio aqui
            
            self.publisher_jugada.publish(True)
            contador_movimientos=0
            #print("jugada realizada")
            print('Llevo ' + str(contador_piezas) + ' piezas')
            #print(str(self.valores_piezas))
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









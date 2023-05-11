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

global contador
contador = 0
global msg

class few_poses:
    def __init__(self):

        global contador
        global turno
        turno=""
        
        self.posicion_camara = Twist()
        self.posicion_articular = Float64MultiArray()
        
        self.posicion_camara= self.create_twist([0.307, 0.192, 0.372, -3.119, -0.022, -1.215])
        self.posicion_articular=[-0.002, -1.281, 0.544, -1.345, -1.522, -0.267]

        self.poses=('open','close',self.posicion_articular, 'open', 'close', self.posicion_camara, 'open', 'close','open')

        #buena posicion [0.3,0.0,0.3,-3.14,0,1.9]  
	    
        self.publisher_articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=1) 
	
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
        self.publisher_finish = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
	
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1) 
	


        self.send_pose = Twist()
        self.send_articular = Float64MultiArray()


    def turn_callback(self, data):


        global turno
        turno= data.data
        if(turno == "robot" or turno == "ROBOT"):
            self.publisher_finish.publish(True)
        
	

    def pose_callback(self, data):

        global contador
        global turno
        #print(turno)
        #print("El valor es " + str(data.data == 1) + " con data = " +str(data.data))
        if(turno == "robot" or turno == "ROBOT"):
            if(contador< len(self.poses)):
                if (data.data == True):
                        #print (contador)
                            
                        if(isinstance(self.poses[contador], str)): #si es open o close
                            if(str(self.poses[contador]) == "open"):
                                self.publisher_gripper.publish("open")
                                print("open")
                            else:
                                self.publisher_gripper.publish("close")
                                print("close")
                                
                                
                        elif("Twist" in str(type(self.poses[contador]))):  #self.poses[contador]._type

                            #self.send_pose=self.create_twist(valores)
                            print("Twist")

                            #print(self.send_pose)
                            self.publisher_pose.publish(self.poses[contador])
                            
                            
                        else: #enviar la pose a la que tiene que ir el robot 

                            #print("float" + str(self.poses[contador]))
                            print("Articular")
                            self.send_articular.data= self.poses[contador]
                            self.publisher_articular.publish(self.send_articular)

                        contador = contador +1
                else:
                        #rospy.loginfo(rospy.get_caller_id() + " Esperando")
                        print("Esperando")
            else:
                self.publisher_turn.publish("jugador")
                contador=0
                #time.sleep(500)

                #rospy.signal_shutdown("Shutting Down")

        else:
            contador=0

    
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
    rospy.init_node('articular_poses') #_node
    obj=few_poses()
   
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









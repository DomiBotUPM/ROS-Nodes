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

global contador
contador = 0
global numpiezas
numpiezas=7
global msg

class pieces:
    def __init__(self):

        global contador
        global numpiezas

        turno=""
    	
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1) 
        self.publisher_init = rospy.Publisher('/play_robot/init', String, queue_size=10)


    def init_callback(self, data):



        global contador
        global numpiezas
        if (data.data == 'init' or data.data == 'INIT'):
            while(contador < numpiezas):
            
                   ## Llamar a un poses_and_gripper actualizado con las coordenadas de la visión  ##
                   print("holi")
                   time.sleep(1)
                   contador= contador+1
                   
                   
                   
                   
            self.publisher_init.publish("finish")
            contador=0
        else:
            print("Finalizada la recogida de piezas")
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









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


class valores_piezas_robot:
    def __init__(self):

        #esto se define por vision
        self.valores_piezas=[[True,3,5],[True,1,5],[True,0,2],[True,3,3],[True,6,4],[True,3,0],[True,0,6]]
        self.valores_piezas_robot = [False, 0, 0, 0, 0, 0, True, 0, 0,True, 0, 0, True, 0, 0, True, 0, 0, True, 0, 0]
            
        #publishers
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1)
        self.publisher_valores_piezas = rospy.Publisher('/posiciones_piezas_robot', Float64MultiArray, queue_size=10)
        
        #subscribers
        self.publisher_valores_nuevas_piezas_robot = rospy.Publisher('/vision/valores_nuevas_piezas', Float64MultiArray, queue_size=1)
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)
        
        
        #init values
        
        self.valores_piezas_robot_send = Float64MultiArray()
        
        
    def init_callback(self,data):
        self.init=data.data
        
        if(self.init == "finish" or self.init == "FINISH"):
            
            #hacer vision para obtener estos valores
            for i in range(len(self.valores_piezas)):
                self.valores_piezas_robot[i*3] = self.valores_piezas[i][0]
                self.valores_piezas_robot[i*3+1] = self.valores_piezas[i][1]
                self.valores_piezas_robot[i*3+2] = self.valores_piezas[i][2]
                
            self.valores_piezas_robot_send.data=self.valores_piezas_robot
            
            self.publisher_valores_nuevas_piezas_robot.publish(self.valores_piezas_robot_send)
        
    
    def turn_callback(self,data):
        if(data.data == "evaluando"):
            self.send_values.data=[random.randint(0,6),random.randint(0,6)]
            self.publisher_valores_nuevas_piezas_robot.publish(self.send_values)
            self.publisher_turn.publish("jugador")
            
        
                  
                     
if __name__ == '__main__':
    
    rospy.init_node('get_all_robot_pieces_values') 
    obj=valores_piezas_robot()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









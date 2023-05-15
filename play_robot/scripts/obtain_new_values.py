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


class valores_nuevas_piezas:
    def __init__(self):


        #publishers

        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1)
        self.publisher_valores_nuevas_piezas_robot = rospy.Publisher('/vision/valores_nuevas_piezas', Float64MultiArray, queue_size=1)
        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10
        self.send_values = Float64MultiArray()
        
    def turn_callback(self,data):
        if(data.data == "evaluando"):
            self.send_values.data=[random.randint(0,6),random.randint(0,6)]
            self.publisher_valores_nuevas_piezas_robot.publish(self.send_values)
            self.publisher_turn.publish("jugador")
            
        
                  
                     
if __name__ == '__main__':
    
    rospy.init_node('obtain_new_values') 
    obj=valores_nuevas_piezas()
    
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









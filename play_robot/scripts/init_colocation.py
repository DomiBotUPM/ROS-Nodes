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
from std_msgs.msg import Float64MultiArray
from moveit_commander.conversions import pose_to_list
import time

global contador
contador = 0
global msg

class few_poses:

    def __init__(self):

        self.posicion_camara= [0.221, -1.144, 0.165, -0.594, -1.565, 4.555]
    
        self.posicion_pieza_robar = self.create_twist([0.264, 0.346, 0.173, 3.089, -0.015, -1.214])
        
        self.poses =('open', self.posicion_camara)





	    #buena posicion [0.3,0.0,0.3,-3.14,0,1.9]
        self.subscriber_init = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1)
	    
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
        self.publisher_finish_go_to_pose = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=1)
        
        self.publisher_articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=1)
        self.send_pose = Twist()
        self.send_articular = Float64MultiArray()


    def init_callback(self, data):
    
        if(data.data == "init" or data.data =="INIT" ):
            self.init="init"
            self.publisher_finish_go_to_pose.publish(True)
            


    

    def pose_callback(self, data):

        global contador
        if(self.init=="init"):
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
                                
                        elif("Twist" in str(type(self.poses[contador]))): #enviar la pose a la que tiene que ir el robot           

                            self.send_pose=self.poses[contador]

                            print("pose")
                            self.publisher_pose.publish(self.send_pose)
                            
                        else:
                            print("Articular")
                            self.send_articular.data= self.poses[contador]
                            self.publisher_articular.publish(self.send_articular)


                        contador = contador +1
                else:
                        print("Esperando")
            else:
                self.init="no"
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
    rospy.init_node('poses_and_gripper') #_node
    obj=few_poses()
   
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









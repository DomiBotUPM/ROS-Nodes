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

class few_poses:
    def __init__(self):

        global contador
        global turno
        turno=""
    # conjuntos de acciones
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],"open",[0.3,0.3,0.3,-3.1,-0.1,0],"close","open","close",[0.3,0.2,0.3,-3.1,-0.1,0],"open","close" ,[0.3,0.2,0.3,-3.14,0,1.6])

	#self.poses=([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14]) #este era
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.25,0.38,-3.2,0.5,0.3])
        
        #self.poses=("open",[-0.173,0.3168,0.324,3.02,0.05,0.302], [-0.173,0.3168,0.224,3.02,0.05,0.302], "close",[-0.173,0.3168,0.324,3.02,0.05,0.302] )

	#self.poses=([0.322, 0.12, 0.276, -3.012, -0.046, -1.201], "open" ,[0.322, 0.12, 0.173, -3.012, -0.046, -1.201], "close", [0.39, 0.118, 0.339, -2.514, -0.243, -1.273], [0.219, -0.199, 0.147, 3.129, 0.004, -2.805], [0.217, -0.278, 0.16, -3.116, 0.002, -2.715], [0.217, -0.278, 0.199, -3.116, 0.002, -2.715],[0.217, -0.34, 0.199, -3.116, 0.002, -2.715],  [0.322, 0.12, 0.21, -3.012, -0.046, -1.201] ,  [0.322, 0.12, 0.173, -3.012, -0.046, -1.201], "open", [0.322, 0.12, 0.276, -3.012, -0.046, -1.201])
	
        self.poses1=([0.03, 0.409, 0.274, 3.044, 0.027, 1.928],'open', [0.268, 0.263, 0.25, 3.098, 0.025, -1.142],[0.268, 0.263, 0.177, 3.098, 0.025, -1.142],'close',[0.268, 0.263, 0.25, 3.098, 0.025, -1.142], [0.171, 0.277, 0.194, -3.111, 0.013, 0.36],[0.166, 0.317, 0.19, -3.085, -0.025, 0.343],[0.166, 0.317, 0.228, -3.056, -0.047, 0.335],[0.16, 0.391, 0.228, -3.113, 0.024, 0.36],[0.03, 0.409, 0.274, 3.044, 0.027, 1.928],[-0.006, 0.425, 0.25, -3.14, 0.047, 2.079], [-0.006, 0.425, 0.175, -3.14, 0.047, 2.079],'open',[0.03, 0.409, 0.274, 3.044, 0.027, 1.928])
        
        self.poses2=([0.307, 0.192, 0.372, -3.119, -0.022, -1.215],'close','open')
        
        self.poses3 = ([0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[-0.036, 0.53, 0.176, 3.122, 0.01, -1.105],[0.009, 0.4, 0.172, -3.12, 0.011, 1.999], [-0.013, 0.476, 0.173, -3.101, 0.037, 1.929], [-0.048, 0.517, 0.172, 3.103, -0.007, 2.015])
        
        self.poses = ([0.307, 0.192, 0.372, -3.119, -0.022, -1.215], [0.106, 0.382, 0.35, -3.063, -0.035, 0.021], [-0.027, 0.474, 0.314, -3.13, 0.014, 1.979], [0.057, 0.522, 0.172, 3.088, -0.013, 1.987],[0.063, 0.464, 0.171, 3.137, 0.03, 2.0],[0.076, 0.401, 0.173, -3.082, 0.013, 2.015],[0.076, 0.302, 0.174, -3.07, 0.028, 1.886], [-0.007, 0.521, 0.171, 3.109, -0.026, 1.89],[-0.007, 0.461, 0.171, 3.109, -0.026, 1.89],[-0.007, 0.401, 0.171, 3.109, -0.026, 1.89], [-0.057, 0.521, 0.171, 3.109, -0.026, 1.89],[-0.027, 0.474, 0.314, -3.13, 0.014, 1.979])





	#buena posicion [0.3,0.0,0.3,-3.14,0,1.9]
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.publisher_turn = rospy.Publisher('/play_robot/turn', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
	
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1) 
	


        self.send_pose = Twist()


    def turn_callback(self, data):


        global turno
        turno= data.data
        
	

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
                        else: #enviar la pose a la que tiene que ir el robot 
                        
                                  
                            '''self.send_pose.linear.x=self.poses[contador][0]
                            self.send_pose.linear.y=self.poses[contador][1]
                            self.send_pose.linear.z=self.poses[contador][2]
                           
                            self.send_pose.angular.x=self.poses[contador][3]
                            self.send_pose.angular.y=self.poses[contador][4]
                            self.send_pose.angular.z=self.poses[contador][5]'''
                            valores=[self.poses[contador][0],self.poses[contador][1],self.poses[contador][2],self.poses[contador][3],self.poses[contador][4],self.poses[contador][5]]
                            self.send_pose=self.create_twist(valores)

                            print(self.send_pose)
                            self.publisher_pose.publish(self.send_pose)

                        contador = contador +1
                else:
                        rospy.loginfo(rospy.get_caller_id() + " Esperando")
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
    rospy.init_node('poses_and_gripper') #_node
    obj=few_poses()
   
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









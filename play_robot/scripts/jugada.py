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
    def __init__(self,jugada_poses):

        global contador
        global finish_jugada
        #turno=""
    # conjuntos de acciones
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],"open",[0.3,0.3,0.3,-3.1,-0.1,0],"close","open","close",[0.3,0.2,0.3,-3.1,-0.1,0],"open","close" ,[0.3,0.2,0.3,-3.14,0,1.6])

	    self.poses=jugada_poses #([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14])
        #self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.25,0.38,-3.2,0.5,0.3])

	#buena posicion [0.3,0.0,0.3,-3.14,0,1.9]
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1)
        
        self.publisher_jugada = rospy.Publisher('/play_robot/jugada/finish', Bool, queue_size=10)
	
        self.subscriber_jugada = rospy.Subscriber('/play_robot/jugada/finish', Bool, self.jugada_callback, queue_size=1) 
	


        self.send_pose = Twist()


    def jugada_callback(self, data):


        global finish_jugada
        finish_jugada= data.data
        

    def pose_callback(self, data):

        global contador
        global finish_jugada


        if(finish_jugada):
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
                            self.send_pose.linear.x=self.poses[contador][0]
                            self.send_pose.linear.y=self.poses[contador][1]
                            self.send_pose.linear.z=self.poses[contador][2]
                           
                            self.send_pose.angular.x=self.poses[contador][3]
                            self.send_pose.angular.y=self.poses[contador][4]
                            self.send_pose.angular.z=self.poses[contador][5]

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
        
	
if __name__ == '__main__':
    #global msg
    rospy.init_node('poses_and_gripper') #_node
    jugada_poses  = ([0.3,0.2,0.3,-3.14,0,0.3],[0.3,0.0,0.3,-3.14,0,1.9],[0.3,0.2,0.3,-3.14,0,-1.3],[0.3,0.2,0.3,-3.14,0,-2.9])#,[0.3,0.2,0.3,-3.14,0,-3.14])
    obj=few_poses(jugada_poses)
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









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
global msg

class few_poses:
    def __init__(self):

        global contador
        global turno
        turno=""
    # conjuntos de acciones
        self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],"open",[0.3,0.3,0.3,-3.1,-0.1,0],"close","open","close",[0.3,0.2,0.3,-3.1,-0.1,0])
        
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
    obj=few_poses()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









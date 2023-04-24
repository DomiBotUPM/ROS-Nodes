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

global contador
contador = 0
global msg

class few_poses:
    def __init__(self):

        global contador
        self.poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],[0.3,0.3,0.3,-3.1,-0.1,0])
        
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=10)
           
        self.publisher_gripper = rospy.Publisher('/gripper/command', String, queue_size=10)

        self.subscriber_finish = rospy.Subscriber('/go_to_pose/finish', Bool, self.pose_callback, queue_size=1) 
        #contador = 0
        self.send_pose = Twist()
        '''
        self.send_pose = Twist() #geometry_msgs.msg.

        self.send_pose.linear.x=self.poses[contador][0]
        self.send_pose.linear.y=self.poses[contador][1]
        self.send_pose.linear.z=self.poses[contador][2]
       
        self.send_pose.angular.x=self.poses[contador][3]
        self.send_pose.angular.y=self.poses[contador][4]
        self.send_pose.angular.z=self.poses[contador][5]

        print(self.send_pose)
        
        self.publisher_pose.publish(self.send_pose)
        '''

    def pose_callback(self, data):

        global contador
        #print("El valor es " + str(data.data == 1) + " con data = " +str(data.data))

        if (data.data == True):
            #print (contador)
            
            print (contador)
            if(contador % 2):

                self.publisher_gripper.publish("open")

            else:
                self.publisher_gripper.publish("close")
            
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

    
        
	
if __name__ == '__main__':
    #global msg
    rospy.init_node('few_poses') #_node
    obj=few_poses()
    
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time

class Mover:
    def __init__(self):
	

        self.publisher_current_pose = rospy.Publisher('/move_robot/current_pose', Pose, queue_size=10)
        self.publisher_pose = rospy.Publisher('/go_to_pose/goal', Twist, queue_size=1)
        
        self.subscriber_dale = rospy.Subscriber('/dale', Bool, self.dale_callback, queue_size=1)
        
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)


        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

       
        self.actual_pose = Twist()
            
        #print("============ Printing robot state")
        articular_state=self.robot.get_current_state()

        state=self.move_group.get_current_pose().pose
        #print(str(state))
        self.publisher_current_pose.publish(state)
        #print(state.joint_state.position)
        
    	orientation_list = [state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        
        self.actual_pose.linear.x=state.position.x
        self.actual_pose.linear.y=state.position.y
        self.actual_pose.linear.z=state.position.z
        
        self.actual_pose.angular.x=roll
        self.actual_pose.angular.y=pitch
        self.actual_pose.angular.z=yaw
        #print(str(self.actual_pose))
        self.publisher_pose.publish(self.actual_pose)
        
        self.contador = 0 
        self.posiciones_piezas_robot=([0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0])
        
    def dale_callback(self,data):
        print(data._type)
        
        prueba = Twist()
        
        #print("Prueba tipo: " + str(prueba._type))
        if(data.data):

            state=self.move_group.get_current_pose().pose
            articular_state=self.robot.get_current_state()
            #print(str(state))
            self.publisher_current_pose.publish(state)
            #print(state.joint_state.position)
            
            orientation_list = [state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

            print("Dale")
            array=[ round(state.position.x,3), round(state.position.y,3), round(state.position.z,3),round(roll,3),round(pitch,3),round(yaw,3)]
            print("Cartesianas: " + str(array))
            #articular_state.joint_state.position
            array_articulares=[round(articular_state.joint_state.position[0],3), round(articular_state.joint_state.position[1],3), round(articular_state.joint_state.position[2],3), round(articular_state.joint_state.position[3],3), round(articular_state.joint_state.position[4],3), round(articular_state.joint_state.position[5],3)]
            print("Articulares: " +str(array_articulares))
            #self.posiciones_piezas_robot[self.contador][0] = [state.position.x,state.position.y,state.position.z,roll,pitch,yaw]
            '''self.posiciones_piezas_robot[self.contador][0] = round(state.position.x,3)
            self.posiciones_piezas_robot[self.contador][1] = round(state.position.y,3)
            self.posiciones_piezas_robot[self.contador][2] = round(state.position.z,3)
            self.posiciones_piezas_robot[self.contador][3] = round(roll,3)
            self.posiciones_piezas_robot[self.contador][4] = round(pitch,3)
            self.posiciones_piezas_robot[self.contador][5] = round(yaw,3)'''
            
            #print(self.posiciones_piezas_robot[self.contador])
            #self.contador = self.contador +1
            '''if self.contador == 6:
                print(self.posiciones_piezas_robot)'''
                
            self.posicion_camara = Twist()
            self.posicion_articular = Float64MultiArray()
            
            self.posicion_camara= self.create_twist([0.307, 0.192, 0.372, -3.119, -0.022, -1.215])
            self.posicion_articular=[-0.002, -1.281, 0.544, -1.345, -1.522, -0.267]
            self.poses=('open','close',self.posicion_articular, 'open', 'close', self.posicion_camara, 'open', 'close','open')
            
            '''for i in range(len(self.poses)):
                print(str(self.poses[i]) + str( type(self.poses[i])))'''
       
            
        
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
    
    rospy.init_node('get_pose') #_node
    obj=Mover()

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









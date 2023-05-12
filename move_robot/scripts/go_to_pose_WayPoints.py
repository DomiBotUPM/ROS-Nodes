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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Mover:
    def __init__(self):
	
        self.publisher_current_pose = rospy.Publisher('/move_robot/current_pose', Pose, queue_size=10)
                
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)


        #para el robot monitored_planning_scene
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        
        self.publisher_finish = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=10)
       # self.publisher_finish.publish(False)
        '''
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("")print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        '''
        self.subscriber = rospy.Subscriber('/go_to_pose/goal', Twist, self.pose_callback, queue_size=1)

    def pose_callback(self, data):

        #waypoints
        #waypoints = []

        #objetivo1
        self.publisher_finish.publish(False)
        
        '''state = self.move_group.get_current_pose().pose
        
        #mover x y
        orientation_list = [state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        objetivo1 =[data.linear.x, data.linear.y, state.position.z, state.orientation.x, roll, pitch, yaw]
        
        #waypoints.append(copy.deepcopy(objetivo1))
        
        #objetivo2
        state = self.move_group.get_current_pose().pose
        #(rz,ryz,rz,rw) = quaternion_from_euler(data.angular.x,data.angular.y,data.angular.z)
        
        objetivo2 =[state.position.x,state.position.y,state.position.z,data.angular.x,data.angular.y,data.angular.z] #mover angulos
        
        #waypoints.append(copy.deepcopy(objetivo2))
        
        #objetivo3
        state = self.move_group.get_current_pose().pose
        
        objetivo3 =[state.position.x, state.position.y, data.linear.z, data.angular.x,data.angular.y,data.angular.z] #mover z
        
        #waypoints.append(copy.deepcopy(objetivo3))
        #print(waypoints)
        #(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0) 
        #waypoints,   # waypoints to follow
        #0.01,        # eef_step
        #0.0)         # jump_threshold'''
                                   
        '''self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()    
        time.sleep(1)
        self.publisher_finish.publish(True)'''


        '''self.move_group.set_pose_target(objetivo1)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.move_group.set_pose_target(objetivo2)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        self.move_group.set_pose_target(objetivo3)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()'''

        x=data.linear.x
        y=data.linear.y
        z=data.linear.z 

        rx=data.angular.x
        ry=data.angular.y
        rz=data.angular.z

        self.move_group.set_pose_target([x,y,z+0.05,rx,ry,rz])
        plan = self.move_group.go(wait=True)
        self.move_group.clear_pose_targets()   
        
        self.move_group.set_pose_target([x,y,z,rx,ry,rz])
        plan = self.move_group.go(wait=True)
        self.move_group.clear_pose_targets()
        time.sleep(1)
        self.publisher_finish.publish(True)
        
        #point to point
        
        self.publisher_finish.publish(False)
        #move_group.set_pose_target(pose_goal)
        x=data.linear.x
        y=data.linear.y
        z=data.linear.z 

        rx=data.angular.x
        ry=data.angular.y
        rz=data.angular.z

        self.move_group.set_pose_target([x,y,z,rx,ry,rz])
        #self.move_group.set_pose_target([0.21, 0.32, 0.205, -0.1 ,-3.1, 0.0 ])

        plan = self.move_group.go(wait=True)
	
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()    
        time.sleep(1)
        self.publisher_finish.publish(True)
        
        #print("============ Printing robot state")
        #state=self.robot.get_current_state()
        state=self.move_group.get_current_pose().pose
        #print(str(state))
        self.publisher_current_pose.publish(state)
        #print(state.joint_state.position)
        '''
        
    	
        
	
if __name__ == '__main__':
    rospy.init_node('go_to_pose') #_node
    obj=Mover()
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass







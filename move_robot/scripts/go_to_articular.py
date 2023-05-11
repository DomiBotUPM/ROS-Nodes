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
        self.subscriber = rospy.Subscriber('/go_to_articular/goal', Float64MultiArray, self.articular_callback, queue_size=1)

    def articular_callback(self, data):

        self.publisher_finish.publish(False)
        
        print(str(data) + " " + str(data._type))
        joint_goal = data.data #[-0.002, -1.281, 0.544, -1.345, -1.522, -0.267]
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        
        time.sleep(1)
        self.publisher_finish.publish(True)

    	
        
	
if __name__ == '__main__':
    rospy.init_node('go_to_articular') #_node
    obj=Mover()
    

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









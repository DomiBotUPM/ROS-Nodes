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

class Mover:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"

        self.move_group = moveit_commander.MoveGroupCommander(group_name)


        #para el robot monitored_planning_scene
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        
        self.publisher_finish = rospy.Publisher('/go_to_pose/finish', Bool, queue_size=10)
       # self.publisher_finish.publish(False)

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        self.subscriber = rospy.Subscriber('/go_to_pose/goal', Twist, self.pose_callback, queue_size=1)

    def pose_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data))
        """
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.0
        pose_goal.orientation.y = 0 #-0.707
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 1 #0.707

        pose_goal.position.x = 0.3
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.5
        """
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
        self.publisher_finish.publish(True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()    

    
        
	
if __name__ == '__main__':
    obj=Mover()
    rospy.init_node('go_to_pose') #_node

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass









#!/usr/bin/env python
import rospy
from ur_msgs.srv import *
from ur_msgs.msg import IOStates

class Gripper:
    def __init__(self, service='/ur_hardware_interface/set_io'):
        rospy.wait_for_service(service)
        try:
            self.set_io = rospy.ServiceProxy(service, SetIO)
            print('Connected with service')
        except rospy.ServiceException as e:
            print('Service unavailable: %s'%e)
    def open(self):
        #self.set_io(1, 17, 0)
        #self.set_io(1, 16, 0)
        rospy.sleep(0.1)
        self.set_io(1, 16, 1)
        rospy.sleep(0.1)
        self.set_io(1, 16, 0)
        
    def close(self):
        #self.set_io(1, 16, 0)
        #self.set_io(1, 17, 0)
        rospy.sleep(0.1)
        self.set_io(1, 17, 1)
        rospy.sleep(0.1)
        self.set_io(1, 17, 0)
	
    def grip_detect(self):
        """grasp success or not"""
        # grip_detect_pin = 0 
        try:
            return rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates).digital_in_states[0].state             
        except rospy.ServiceException as e:
            print('cannot get io states: %s'%e)

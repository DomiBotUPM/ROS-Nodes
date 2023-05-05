#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from play_robot.srv import *

class turn_node():
    def __init__(self):
        self.turno = "none"
        self.subscriber_turn = rospy.Subscriber('/play_robot/turn', String, self.turn_callback, queue_size=1) 

        s = rospy.Service('turn_service', turn, self.handle_turn)

    def turn_callback(self, data):

        self.turno = str(data.data)

    def handle_turn(self,req):
        return turnResponse(self.turno)


    '''def turn_server():
        rospy.init_node('turn_server')
        
        print("Ready to send turn")
        rospy.spin()'''

if __name__ == "__main__":

    rospy.init_node('turn_service') #_node
    obj=turn_node()
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass

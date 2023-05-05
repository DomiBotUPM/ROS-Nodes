#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from play_robot.srv import *

class collect_node():
    def __init__(self):
        self.collect = "none"
        self.subscriber_collect = rospy.Subscriber('/play_robot/init', String, self.init_callback, queue_size=1) 

        s = rospy.Service('collect_service', collect, self.handle_collect)

    def init_callback(self, data):

        self.collect = str(data.data)

    def handle_collect(self,req):
        return collectResponse(self.collect)



if __name__ == "__main__":

    rospy.init_node('collect_service') #_node
    obj=collect_node()
    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass

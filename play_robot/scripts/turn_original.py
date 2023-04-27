#!/usr/bin/env python

import sys
import rospy
from play_robot.srv import *


def handle_turn(req):
        return turnResponse("humano")


def turn_server():
        rospy.init_node('turn_server')
        s = rospy.Service('turn_service', turn, handle_turn)
        print("Ready to send turn")
        rospy.spin()

if __name__ == "__main__":

    turn_server()


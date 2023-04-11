#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):

    import subprocess
    commands = data.data.split("&&")
    pub = rospy.Publisher('/test', String, queue_size=10)
    pub.publish('Hi')
    for command in commands:
        subprocess.call(command, shell = True) 
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('launcher')

    rospy.Subscriber("launch_signal", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

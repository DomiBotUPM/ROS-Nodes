#!/usr/bin/env python
from geometry_msgs.msg import Twist
import sys
import copy
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64MultiArray



        
class tuple_pub:

    def __init__(self):
        self.pub = rospy.Publisher('/posiciones_piezas_robot', Float64MultiArray, queue_size=10)
        self.pubstr = rospy.Publisher('/posiciones_piezas_str', String, queue_size=10)
        self.subscriber = rospy.Subscriber('/dale', String, self.callback, queue_size=1)
        
    def callback(self, data):
        array = [False, 2, 6 ,True, 0, 2 , True, 4, 5 ,True, 2, 3,True, 5, 0,True, 3, 3,True, 0, 2]
        
        data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
        
        data_to_send.data = array # assign the array with the value you want to send
        
        self.pub.publish(data_to_send)
        
        self.pubstr.publish("holi")
        
        print(data_to_send)
        
        #print(data_to_send.data)
        
        
        #subscriber = rospy.Subscriber('/posiciones_piezas', Float64MultiArray, self.callback, queue_size=1)
    #def callback(self,data):
        #print(data)
    
    
if __name__ == '__main__':
    
    rospy.init_node('tuple_pub') #_node
    obj=tuple_pub()

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass


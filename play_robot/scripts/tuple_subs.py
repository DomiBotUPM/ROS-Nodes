#!/usr/bin/env python
from geometry_msgs.msg import Twist
import sys
import copy
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64MultiArray



        
class tuple_subs:
    def __init__(self):

        subscriber = rospy.Subscriber('/posiciones_piezas_robot', Float64MultiArray, self.callback, queue_size=1)
        self.posiciones = ([False, 0, 0], [0, 0, 0], [True, 0, 0 ],[True, 0, 0],[True, 0, 0],[True, 0, 0],[True, 0, 0])
        
    def callback(self,data):
        datos = data.data
	    
        for i in range(len(datos)/3):
            self.posiciones[i][0]=datos[i*3]
            self.posiciones[i][1]=datos[i*3+1]
            self.posiciones[i][2]=datos[i*3+2]
                
        #print(data.data)
        print(self.posiciones)
    
if __name__ == '__main__':
    
    rospy.init_node('tuple_subs') #_node
    obj=tuple_subs()

    try:
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python
from geometry_msgs.msg import Twist
import sys
import copy
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float64MultiArray

'''
poses=([0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],"open",[0.3,0.3,0.3,-3.1,-0.1,0],"close")
print(poses[1])
print(isinstance(poses[1], str))
print(poses[2])
print(isinstance(poses[2], str))
print(len(poses))
print(poses[len(poses)-1])

matriz= [[False,2,3],[True,5,6]]
matriz[1]=[False,6,3]
print(matriz)

valores_piezas=[[0,0,0],[1,0,0],[2,0,0],[3,0,0],[4,0,0],[5,0,0],[5,0,0]]

print(valores_piezas)

print(str(valores_piezas))

for i in range(len(valores_piezas)):
    print(valores_piezas[i][0])
    print(i)
    if(valores_piezas[i][0] == 2):
        print("entra")
        break

'poses="[0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],\"open\",[0.3,0.3,0.3,-3.1,-0.1,0],\"close\",\"open\",\"close\",[0.3,0.2,0.3,-3.1,-0.1,0],\"open\",\"close\" ,[0.3,0.2,0.3,-3.14,0,1.6]"

print(poses)



print(poses.split(","))'''


rospy.init_node('tuple') #_node

'''array = [[True, 2, 6], [True, 0, 2], [True, 4, 5], [True, 2, 3], [True, 5, 0], [True, 3, 3], [True, 0, 2]]
pub = rospy.Publisher('/posiciones_piezas', Float64MultiArray, queue_size=10)
pubstr = rospy.Publisher('/posiciones_piezas_str', String, queue_size=10)
data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
data_to_send.data = array # assign the array with the value you want to send
pub.publish(data_to_send)
pubstr.publish(str(data_to_send))
print(array)
print(data_to_send)
print(str(data_to_send))




valores_piezas=[[True,3,5],[True,1,5],[True,0,2],[True,3,3],[True,6,4],[True,3,0],[True,0,6]]
print("INICIAL "+ str(valores_piezas))
valores_piezas.append([False,8,8])
print("FINAL "+ str(valores_piezas))'''


#ir a una pose articular
arti=0
if(arti==1):
    articular = rospy.Publisher('/go_to_articular/goal', Float64MultiArray, queue_size=10)
    posicion_camara_tablero= [0.221, -1.144, 0.165, -0.594, -1.565, 4.555] #[0.094, -1.182, 0.514, -0.959, -1.578, -0.324] 
    posicion_camara_piezas_robot= [1.265, -0.998, 0.433, -0.946, -1.6, -2.253] #[1.377, -1.102, 0.48, -0.917, -1.576, -0.544] 
    send_articular = Float64MultiArray()
    send_articular.data= posicion_camara_piezas_robot
    print(send_articular)
    articular.publish(send_articular)


publisher_init = rospy.Publisher('/play_robot/init', String, queue_size=10)
publisher_init.publish("finish")
print("publish")



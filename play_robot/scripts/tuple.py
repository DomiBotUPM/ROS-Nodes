#!/usr/bin/env python
from geometry_msgs.msg import Twist



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

'''poses="[0.3,0.1,0.3,-3.1,-0.1,0],[0.3,0.2,0.3,-3.1,-0.1,0],\"open\",[0.3,0.3,0.3,-3.1,-0.1,0],\"close\",\"open\",\"close\",[0.3,0.2,0.3,-3.1,-0.1,0],\"open\",\"close\" ,[0.3,0.2,0.3,-3.14,0,1.6]"

print(poses)



print(poses.split(","))'''

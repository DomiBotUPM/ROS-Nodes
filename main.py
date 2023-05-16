import os
from vision.vision_interface import DominoVision
import cv2 as cv

domino_vision = DominoVision(visualize=False, verbose=False)

# Probar directamente desde la camara
# domino_vision.test_with_video(channel=1)
#domino_vision.view_video(channel=2)

#domino_vision.test_with_video(channel=2)
#piezas=[]

capture = cv.VideoCapture(2)

ret, frame = capture.read()

#piezas = domino_vision.pieces_recognition(capture,480,piezas)
#print(piezas)

#cv.imshow('Video', frame)

#cv.waitKey(0)
#mat= cv.Mat()

size = frame.shape[0]*frame.shape[1]
detections = domino_vision.pieces_detection(frame, size)
recognitions =  domino_vision.pieces_recognition(frame, size, pieces=detections)

recognitions2 = domino_vision.ordenar_piezas(recognitions)

for pieza in recognitions2:
    #print([pieza.center[0], pieza.center[1], pieza.angle, pieza.dots[0], pieza.dots[1]])
    print([pieza.dots[0], pieza.dots[1]])

#domino_vision.test_with_image(frame)

# print('a')

domino_vision.save_img(frame)

# Probar con imagenes

# path_dir = os.path.abspath("vision/fotos_ur3/")
# i = 0
# for file in os.listdir(path_dir)[:]:
#     filename = os.path.join(path_dir, file)
#     domino_vision.test_with_image(filename)
#     cv.waitKey(0)
    

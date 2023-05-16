import cv2 as cv
import imutils
from typing import Tuple, List
import numpy as np
import math

from .preprocessing import preprocessing_img
from .pieces_detection_v1 import pieces_detection

def _piece_recognition_from_mask(img: cv.Mat, angle: float, preprocess=False, verbose=False) -> Tuple[str, list, list]:
    """Reconocimiento de pieza que ha sido aislada a partir de una máscara

    Args:
        img: Imagen con la mascara aplicada.
        angle (float): Angulo de rotacion
        preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to False.
        verbose (bool, optional): Mostrar mensajes de seguimiento. Defaults to False.

    Returns:
        Tuple[str, list, list]: Tipo de pieza, contornos de los puntos internos, y contorno de la línea separadora.
    """
    # Preprocesamiento de la imagen
    if preprocess:
        img = preprocessing_img(img, visualize=False)
    # Contornos internos a la pieza
    orig_contours, _ = cv.findContours(img, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
    # Si no hay contornos internos (salvo el de la propia pieza), entonces la pieza está del revés
    if len(orig_contours) <= 1:
        return "reves", [], []
    if verbose:
        print(f"Hay {len(orig_contours)} contornos internos")
    # Medimos el área de la pieza para tenerla como referencia
    piece_area = cv.contourArea(orig_contours[0])
    # Diferenciar entre los puntos y la línea separadora
    dot_contours = []
    line_contour = []
    for contour in orig_contours[1:]:
        center_i, size_i, angle_i = cv.minAreaRect(contour)
        box = np.int64(cv.boxPoints((center_i, size_i, angle_i)))
        if size_i[0] == 0:
            # print(f"Alguna medida es 0: {size_i}. Box: {box}")
            continue
        ratio = size_i[1]/size_i[0]
        if ratio > 0.5 and ratio < 2 and size_i[0]*size_i[1] < 0.5*piece_area:
            dot_contours.append(box) # punto
            if verbose:
                print(f"Punto con ratio: {ratio}")
        else:
            line_contour = [box] # línea separadora
            if verbose:
                print(f"Línea con ratio: {ratio}")
    # Giramos la pieza para tenerlo en posición horizontal o vertical
    img = imutils.rotate(img, angle)
    rect_contours, _ = cv.findContours(img, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
    # Comprobamos si está en vertical u horizontal
    (x,y,w,h) = cv.boundingRect(rect_contours[0])
    is_vertical = h/w > 1
     # Definimos si comparamos con x o con y
    index_pos = 0
    if is_vertical:
        index_pos = 1
    # Obtenemos las posiciones de la línea separadora y de los puntos
    ref = (0,0)
    dots = []
    for contour in rect_contours[1:]:
        (x,y,w,h) = cv.boundingRect(contour)
        ratio = h/w
        if ratio < 0.5 or ratio > 2:
            ref = (x,y)
        else:
            dots.append((x,y))
        # print(x,y,w,h, h/w)
        
    # Separamos los puntos en dos grupos
    n_dots_up = len([dot for dot in dots if dot[index_pos] > ref[index_pos]])
    n_dots_down = len([dot for dot in dots if dot[index_pos] < ref[index_pos]])

    # El primer valor siempre debe ser mayor igual que el segundo
    if n_dots_up < n_dots_down:
        aux = n_dots_up
        n_dots_up = n_dots_down
        n_dots_down = aux

    piece_type = f"{n_dots_up}x{n_dots_down}"
    return piece_type, dot_contours, line_contour

def pieces_recognition(img: cv.Mat, size=1e4, preprocess=False, verbose=False, visualize=False) -> List[dict]:
    """Reconocimiento del tipo de pieza utilizando la librería OpenCV. Puede reconocer distintas piezas

    Args:
        img (Mat): Imagen a clasificar
        size (float): Área de la imagen total en píxeles (width*height)
        preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to False.
        verbose (bool): Mostrar mensajes de seguimiento. Defaults to False.
        visualize (bool): Visualizar imágenes intermedias. Defaults to False.
    Returns:
        List[dict]: Lista de piezas reconocidas. Datos de la pieza: contorno, centro en px, (ancho,alto) en px, ángulo de rotación en º y tipo.
    """
    img_i = img.copy()
    # Preprocesamiento de la imagen
    if preprocess:
        processed_img = preprocessing_img(img_i, visualize=False)
    else:
        processed_img = img_i
    detections = pieces_detection(processed_img, size, verbose=verbose, visualize=False)
    
    recognitions = []
    for detection in detections:
        mask = np.zeros(processed_img.shape, np.uint8)
        cv.fillPoly(mask, [detection['contour']], color=(255))
        masked = cv.bitwise_and(processed_img, mask)
        piece_type, dot_contours, line_contour = _piece_recognition_from_mask(masked, detection['angle'], verbose=verbose)
        detection['type_piece'] = piece_type
        recognitions.append(detection)
        if visualize:
            cv.drawContours(img_i,[detection['contour']],0,(255,0,0),thickness=2)
            for c in dot_contours:
                radius_c = round(abs(np.sqrt((c[1][0] - c[0][0])**2 + (c[1][1] - c[0][1])**2))/2)
                center_c = c[0] + (c[2] - c[0])/2
                cx = round(center_c[0])
                cy = round(center_c[1])
                cv.circle(img_i, (cx,cy), radius_c, color=(0,255,0), thickness=1)
            if len(line_contour):
                cv.drawContours(img_i,[line_contour[0]],0,(0,0,255),thickness=1)
            cx = round(detection['center'][0])
            cy = round(detection['center'][1])
            cv.rectangle(img_i, (cx-12, cy-6), (cx+12, cy+6), (255,255,255), thickness=-1)
            cv.putText(img_i, piece_type, (cx-12, cy+3), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,0), 1, cv.LINE_AA)
            cv.imshow("Reconocimiento de piezas", img_i)
        if verbose:
            print(f"Pieza de tipo {piece_type}")
    
    return recognitions
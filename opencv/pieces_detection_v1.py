import cv2 as cv
from typing import List
import numpy as np

from .preprocessing import preprocessing_img

def pieces_detection(img, size: float, preprocess=False, verbose=False, visualize=False) -> List[dict]:
    """Deteccion de las fichas de domino presentes en la zona

    Args:
        img (Mat): Imagen
        size (float): Area de la imagen total en píxeles
        preprocess (bool, optional): Realizar preprocesamiento de la imagen. Defaults to False.
        verbose (bool, optional): Mostrar mensajes de seguimiento. Defaults to False.
        visualize (bool, optional): Visualizar imágenes intermedias. Defaults to False.

    Returns:
        List[dict]: Lista de piezas detectadas. Se indica: contorno rectangular, centro, (ancho,alto) y ángulo de rotación de la pieza en º.
    """
    img_i = img.copy()
    # Preprocesamiento de la imagen
    if preprocess:
        processed_img = preprocessing_img(img_i, visualize=False)
    else:
        processed_img = img_i
        
    # Detectar contornos
    contours, _ = cv.findContours(processed_img, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
    
    if verbose:
        print(f"Hay {len(contours)} contornos")

    # Encontrar solo los contornos que sean de piezas
    detections = []
    for contour in contours:
        area = cv.contourArea(contour)
        # El tamaño mínimo para un punto
        if area < 1.6e-4*size:
            continue
        center, (width,height), angle = cv.minAreaRect(contour)
        ratio = width/height
        # Para que sea una pieza el ancho debe ser la mitad que el alto y debe ser al menos de un tamaño concreto
        if (ratio > 0.45 or ratio < 0.55) and width*height > 7e-3*size:
            box = np.int64(cv.boxPoints((center, (width,height), angle)))
            # detections.append((box, np.round(center,3), (round(width,3), round(height,3)), round(angle,3)))
            detections.append({'contour': box, 'center': np.round(center,3), 'size_px': (round(width,3), round(height,3)), 'angle': angle})
            if visualize:
                cv.drawContours(img_i,[box],0,(255,0,0), thickness=2)
                cv.imshow("Deteccion de piezas", img_i)
        if verbose:
            print(f"Area: {area}")
            print(f"Area del rectangulo: {width}*{height} = {width*height}")
            print(f"Tamaño de referencia para reconocer pieza: {7e-3*size}")
    if verbose:
        print(f"Hay {len(detections)} piezas")
    
    return detections


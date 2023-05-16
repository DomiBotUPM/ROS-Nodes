import cv2 as cv

def piece_recognition(img) -> str:
    """Reconocimiento del tipo de pieza utilizando la librería OpenCV

    Args:
        img (Mat): Imagen a clasificar

    Returns:
        str: Tipo de ficha
    """

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # cv.imshow("Escala de grises", gray)

    # El problema de Canny es que luego detectamos contornos duplicados
    # canny = cv.Canny(gray, 100, 150)
    # cv.imshow("Canny", canny)

    thresh, binarized = cv.threshold(gray, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    cv.imshow(f"Imagen binarizada con umbral: {thresh}", binarized)

    contours, hierarchy = cv.findContours(binarized, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)
    print(hierarchy.shape)
    print(f"Hay {len(contours)} contornos")

    # Primero tratamos el contorno externo
    idx_max_contour, max_contour = max(enumerate(contours), key = lambda x:cv.contourArea(x[1]))
    print(f"El contorno externo tiene {len(contours[idx_max_contour])} lados")
    (x,y,w,h) = cv.boundingRect(max_contour)
    cv.rectangle(img, (x,y), (x+w,y+h), color=(0,255,0), thickness=1)
    cv.imshow("Imagen con contornos", img)
    print(f"Contorno externo de la pieza: Punto de inicio {(x,y)} y área {w}*{h}={w*h}")

    # Si está en horizontal, lo ponemos en vertical
    # También calculamos el ancho de la ficha para obtener medidas relativas de los puntos y de la línea separadora
    is_vertical = h/w > 1
    if is_vertical:
        index_pos = 1 # Definimos si comparamos con x o con y
        ref_piece = w
    else:
        index_pos = 0
        ref_piece = h

    print(f"La referencia de la pieza será: {ref_piece} y dentro de cada punto se tomará el índice {index_pos}")

    # Ahora tratamos los contornos internos
    dots = []
    ref = (0,0)
    for i, contour in enumerate(contours):
        # Ya no tratamos el contorno grande
        if i == idx_max_contour:
            continue
        epsilon = 0.05*cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        # print(f"El contorno tiene {len(approx)} lados")
        (x,y,w,h) = cv.boundingRect(contour)
        
        # Si es vertical comparamos el ancho y si es horizontal el alto
        if is_vertical:
            ref_inner = w
        else:
            ref_inner = h
        
        if ref_inner/ref_piece < 0.2:
            continue
        elif ref_inner/ref_piece >= 0.2 and ref_inner/ref_piece < 0.4: # punto
            dots.append((x,y))
        elif ref_inner/ref_piece > 0.8: # línea separadora
            ref = (x,y)
        
        print(f"Punto de inicio {(x,y)} y área {w}*{h}={w*h}")
        cv.rectangle(img,(x,y), (x+w,y+h), color=(255,0,0), thickness=1)
        cv.imshow("Imagen con contornos", img)
        cv.waitKey(500)

    # Separamos los puntos en dos grupos
    n_dots_up = len([dot for dot in dots if dot[index_pos] > ref[index_pos]])
    n_dots_down = len([dot for dot in dots if dot[index_pos] < ref[index_pos]])

    # El primer valor siempre debe ser mayor igual que el segundo
    if n_dots_up < n_dots_down:
        aux = n_dots_up
        n_dots_up = n_dots_down
        n_dots_down = aux

    type_piece = f"{n_dots_up}x{n_dots_down}"

    return type_piece
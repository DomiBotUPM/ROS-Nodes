# import sys
# sys.path.append("..\vision")

import numpy as np
from .pieza_sencilla import PiezaSencilla
from .domino_game import numeroComun

def piezasVerticalSeguidas(tablero, direccion, ancho_pieza):
    """Una vez tenemos varias piezas en vertical, se pone un limite al numero seguido de piezas en vertical seguidas (3, sin contar dobles) antes de hacer ya un giro.
    Esta funcion calcula cuantas hay seguidas en una misma direccion.
    
        Args:
            tablero (List[pieza_sencilla]): piezas ordenadas del tablero. La primera pieza siempre es la del extremo superior (o izquierdo).
            direccion (string): direccion en la que se estan colocando las piezas: 'arriba' o 'abajo'.
            ancho_pieza: ancho de la pieza. Usado para detectar si dos piezas estan en la misma vertical, siendo este el valor maximo que se pueden desviar.
        Returns:
            np: numero de piezas consecutivas en vertical.
    """
    np = 0
    if direccion == 'arriba':
        tablero.reverse()
    for pieza in tablero:
        # si estan en la misma vertical, se suma 1
        if abs(tablero[0].center[0] - pieza.center[0]) < ancho_pieza:
            # si es doble, no se cuenta
            if pieza.esDoble():
                continue
            else:
                np += 1
                
        # si no estan en la misma vertical, se termina
        else:
            return np
    return np

def proximaDireccionHorizontal(tablero, direccion):
    """Devuelve la direccion (en horizontal) en la que hay que colocar la proxima pieza.
 
    
        Args:
            tablero (List[pieza_sencilla]): piezas ordenadas del tablero. La primera pieza siempre es la del extremo superior (o izquierdo).
            direccion (string): direccion general del tablero en la que se estan colocando las piezas: 'arriba' (extremo 1) o 'abajo' (extremo 2).
        Returns:
            'izquierda' o 'derecha'
    """
    if len(tablero) <= 1:
        return 'derecha'
    elif direccion == 'abajo':
        if tablero[0].center[0] < tablero[1].center[0]:
            return 'izquierda'
        else:
            return 'derecha'
    else:
        if tablero[-1].center[0] < tablero[-2].center[0]:
            return 'izquierda'
        else:
            return 'derecha'

# #######################################
# ######## FUNCIONES IMPORTANTES ########
# #######################################

def colocarPieza(movimiento, limite1, limite2, longitud_pieza, ancho_pieza, tablero):
    """Segun el movimiento calculado en decision_jugada.domino_game.decidirMovimiento(), se dan las coordenadas de la pieza a utilizar (o su indice, si asi se han pasado).
    Si la orden es de robar en vez de jugar, todos los valores se ponen en -1.
    Si es el primer turno del robot, esto puede que funcione bastante mal...
        Args:
            movimiento (Dictionary): conjunto de ordenes que hay que jugar. Los valores son:
            -Comunes:
                'movimiento':       'jugada' si debe colocar una pieza. 'robar' si no hay pieza que pueda ser jugada.
            -Si movimiento['movimiento'] = 'jugada'
                'pieza_tablero':    pieza del tablero a continuacion de la cual el robot va a poner la suya.
                'pieza_robot':      pieza del robot que se desea jugar.
                'direccion':        en caso de que la ficha no se pueda colocar al lado debido a que se rebasan los limites del tablero, se debe colocar 'arriba' (extremo 1 del tablero) o 'abajo' (extremo dos del tablero). 
            limite1:        posiciones x por debajo de las cuales nos acercamos peligrosamente al borde del tablero y es mejor no colocar pieza (salvo doble).
            limite2:        posiciones x por encima de las cuales nos acercamos peligrosamente al borde del tablero y es mejor no colocar pieza (salvo doble).
            ancho_pieza:    ancho de la pieza.
        Returns:
            origen ([x, y]):    coordenadas (x,y) de la pieza que queremos jugar. Si solo usamos su indice, con poner [indice, 0] basta.
            angulo_origen:      angulo (en grados) de la pieza que queremos jugar. Si solo usamos su indice, es bastante indiferente.
            destino ([x, y]):   coordenadas (x,y) del lugar donde queremos colocar la pieza.
            angulo_destino:     angulo (en grados) con el que queremos colocar la pieza.
    """


    RATIO_DIST = 1.2

    if movimiento["movimiento"] == 'robar':
        return [-1, -1], -1, [-1, -1], -1
    
    pieza_robot = movimiento["pieza_robot"]
    pieza_tablero = movimiento["pieza_tablero"]
    proxima_direccion_horizontal = proximaDireccionHorizontal(tablero, movimiento["direccion"])

    origen = pieza_robot.center
    angulo_origen = pieza_robot.angle

    # ------------- PIEZA ROBOT DOBLE ----------------
    if pieza_robot.esDoble():
        # HORIZONTAL + A LA IZQUIERDA
        if pieza_tablero.esHorizontal() and proxima_direccion_horizontal == 'izquierda':
            destino = [pieza_tablero.center[0] - longitud_pieza, pieza_tablero.center[1]]
            angulo_destino = 90
        # HORIZONTAL + A LA DERECHA
        elif pieza_tablero.esHorizontal() and proxima_direccion_horizontal == 'derecha':
            destino = [pieza_tablero.center[0] + longitud_pieza, pieza_tablero.center[1]]
            angulo_destino = 90 
        # VERTICAL + ABAJO
        elif movimiento["direccion"] == 'abajo':
            destino = [pieza_tablero.center[0], pieza_tablero.center[1] + longitud_pieza]
            angulo_destino = 0 
        # VERTICAL + ARRIBA
        else:
            destino = [pieza_tablero.center[0], pieza_tablero.center[1] - longitud_pieza]
            angulo_destino = 0 

    # ------------- PIEZA TABLERO DOBLE ----------------
    elif pieza_tablero.esDoble(): # mas complicado:(
        numero_comun = numeroComun(pieza_robot, pieza_tablero)
            
        # HORIZONTAL + ABAJO
        if pieza_tablero.esHorizontal() and movimiento["direccion"] == 'abajo':
            destino = [pieza_tablero.center[0], pieza_tablero.center[1] + longitud_pieza]
            angulo_destino = 90
        # HORIZONTAL + ARRIBA
        elif pieza_tablero.esHorizontal() and movimiento["direccion"] == 'arriba':
            destino = [pieza_tablero.center[0] + longitud_pieza, pieza_tablero.center[1]]
            angulo_destino = 90 + 180
        # VERTICAL + IZQUIERDA
        elif proxima_direccion_horizontal == 'izquierda':
            # poner a continuacion
            if pieza_tablero.center[0] - longitud_pieza > limite1:
                destino = [pieza_tablero.center[0] - longitud_pieza, pieza_tablero.center[1]]
                angulo_destino = 0 + 180
            # poner abajo 
            elif movimiento["direccion"] == 'abajo': # lio de hacerlo del reves
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] + longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 0
            # poner arriba
            else:
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] - longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 180
        # VERTICAL + DERECHA
        elif proxima_direccion_horizontal == 'derecha':
            # poner a continuacion
            if pieza_tablero.center[0] + longitud_pieza < limite2:
                destino = [pieza_tablero.center[0] + longitud_pieza, pieza_tablero.center[1]]
                angulo_destino = 0 + 0
            # poner abajo 
            elif movimiento["direccion"] == 'abajo': # lio de hacerlo del reves
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] + longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 0
            # poner arriba
            else:
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] - longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 180
                # pieza origen

        if pieza_robot.v1 != numero_comun:
            angulo_destino += 180
        
    # ------------- NO DOBLES ----------------
    else:
        numero_comun = numeroComun(pieza_robot, pieza_tablero)

        # HORIZONTAL + A LA IZQUIERDA -> ¿se podria mirar simplemente con el 'arriba' o 'abajo'? -> NO
        if pieza_tablero.esHorizontal() and proxima_direccion_horizontal == 'izquierda':
            # poner a continuacion
            if pieza_tablero.center[0] - longitud_pieza*RATIO_DIST > limite1:
                destino = [pieza_tablero.center[0] - longitud_pieza*RATIO_DIST, pieza_tablero.center[1]]
                angulo_destino = 0 + 180
            # poner abajo 
            elif movimiento["direccion"] == 'abajo': # lio de hacerlo del reves
                destino = [pieza_tablero.center[0] - ancho_pieza/2, pieza_tablero.center[1] + longitud_pieza]
                angulo_destino = 90 + 0
            # poner arriba
            else:
                destino = [pieza_tablero.center[0] - ancho_pieza/2, pieza_tablero.center[1] - longitud_pieza]
                angulo_destino = 90 + 180
        # HORIZONTAL + A LA DERECHA
        elif pieza_tablero.esHorizontal() and proxima_direccion_horizontal == 'derecha':
            # poner a continuacion
            if pieza_tablero.center[0] + longitud_pieza*RATIO_DIST < limite2:
                destino = [pieza_tablero.center[0] + longitud_pieza*RATIO_DIST, pieza_tablero.center[1]]
                angulo_destino = 0 + 0
            # poner abajo 
            elif movimiento["direccion"] == 'abajo': # lio de hacerlo del reves
                destino = [pieza_tablero.center[0] + ancho_pieza/2, pieza_tablero.center[1] + longitud_pieza]
                angulo_destino = 90 + 0
            # poner arriba
            else:
                destino = [pieza_tablero.center[0] + ancho_pieza/2, pieza_tablero.center[1] - longitud_pieza]
                angulo_destino = 90 + 180
        # VERTICAL + ABAJO
        elif movimiento["direccion"] == 'abajo':
            # poner a la izquierda o derecha
            if piezasVerticalSeguidas(tablero, 'abajo', ancho_pieza) >= 3:
                # poner a la derecha (estas en izquierda)
                if abs(pieza_tablero.center[0] - limite1) < abs(pieza_tablero.center[0] - limite2):
                    destino = [pieza_tablero.center[0] + longitud_pieza, pieza_tablero.center[1] + ancho_pieza/2]
                    angulo_destino = 0 + 0
                # poner a la izquierda (estas en izquierda)
                else:
                    destino = [pieza_tablero.center[0] - longitud_pieza, pieza_tablero.center[1] + ancho_pieza/2]
                    angulo_destino = 0 + 180
            # poner abajo
            else:
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] + longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 0
        # VERTICAL + ARRIBA
        else:
            # poner a la izquierda o derecha
            if piezasVerticalSeguidas(tablero, 'arriba', ancho_pieza) >= 3:
                # poner a la derecha (estas en izquierda)
                if abs(pieza_tablero.center[0] - limite1) < abs(pieza_tablero.center[0] - limite2):
                    destino = [pieza_tablero.center[0] + longitud_pieza, pieza_tablero.center[1] - ancho_pieza/2]
                    angulo_destino = 0 + 0
                # poner a la izquierda (estas en izquierda)
                else:
                    destino = [pieza_tablero.center[0] - longitud_pieza, pieza_tablero.center[1] - ancho_pieza/2]
                    angulo_destino = 0 + 180
            # poner arriba
            else:
                destino = [pieza_tablero.center[0], pieza_tablero.center[1] - longitud_pieza*RATIO_DIST]
                angulo_destino = 90 + 180
        if pieza_robot.v1 != numero_comun:
            angulo_destino += 180

    angulo_origen = angulo_origen % 360

    return origen, angulo_origen, destino, angulo_destino


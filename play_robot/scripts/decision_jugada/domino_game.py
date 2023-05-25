import numpy as np
from .pieza_sencilla import PiezaSencilla

# Logica sencilla para un juego de domino

def distanciaPiezas(pieza1, pieza2, orden):
    """Distancia entre los centros de dos piezas (o piezas sencillas), definida segun una norma del orden especificado.
    
        Args:
            pieza1: pieza.
            pieza2: pieza.
            orden:  orden de la norma.
        Returns:
            distancia entre las piezas.
    """

    c1 = np.array(pieza1.center)
    c2 = np.array(pieza2.center)

    return np.linalg.norm(c1 - c2, ord = orden)

def masCercanos(pieza_elegida, piezas, orden):
    """Distancias de un conjunto de piezas a otra pieza. Ademas, devuelve tambien el orden en el que se encuentran, de menor a mayor distancia.
    
        Args:
            pieza_elegida:  pieza respecto a la que se calculan las distancias del resto de piezas.
            piezas (List[piezasencilla] o List[Piece]): array de piezas de las cuales se van a calcular las distancias. 
            orden:  orden de la norma para calcular las distancias.
        Returns:
            indices_orden:  indices de las piezas ordenadas de menor a mayor distancia.
            distancias:     distancias de las piezas a la pieza elegida (desordenadas).
    """
    distancias = []

    for pieza in piezas:
        distancias.append(distanciaPiezas(pieza_elegida, pieza, orden))

    indices_orden = sorted(range(len(distancias)), key=lambda k: distancias[k]) # stack overflow

    return indices_orden, distancias

def numeroComun(pieza1, pieza2):
    """Numero en comun entre las piezas (sencillas).
    
        Args:
            pieza1 (piezasencilla).
            pieza2 (piezasencilla).
        Returns:
            numero_comun: numero comun a las piezas. -1 si no existe.
    """
    if pieza1.v1 == pieza2.v1:
        return pieza1.v1
    elif pieza1.v1 == pieza2.v2:
        return pieza1.v1
    elif pieza1.v2 == pieza2.v1:
        return pieza1.v2
    elif pieza1.v2 == pieza2.v2:
        return pieza1.v2
    else: 
        return -1
    
def numeroDiferenteExtremo(pieza_extremo, pieza_interior):
    """Dada una pieza en un extremo y una pieza que esta en el interior, devuelve cual es el numero del extremo.
    Para ello, compara los numeros iguales de las piezas, y devuelve el numero contrario, en la pieza del extremo.
    
        Args:
            pieza_extremo (PiezaSencilla).
            pieza_interior (PiezaSencilla).
        Returns:
            numero_diferente_extremo: numero no comun en la pieza del extremo. -1 si hay algun error raro y todos los numeros son distintos.
    """
    if pieza_extremo.v1 == pieza_interior.v1:
        return pieza_extremo.v2
    elif pieza_extremo.v1 == pieza_interior.v2:
        return pieza_extremo.v2
    elif pieza_extremo.v2 == pieza_interior.v1:
        return pieza_extremo.v1
    elif pieza_extremo.v2 == pieza_interior.v2:
        return pieza_extremo.v1
    else: 
        return -1

def extremosTablero(tablero):
    """Devuelve los numeros de los extremos del tablero, es decir, los numeros en los que hay que poner la pieza.
    
        Args:
            tablero (List[piezasencilla]): tablero ordenado de piezas.
        Returns:
            [numero_extremo_1, numero_extremo_2].
    """
    if len(tablero) == 0:
        return []
    elif len(tablero) == 1: # en teoria siempre deberia ser doble pero bueno
        if tablero[0].esDoble():
            return [tablero[0].v1, tablero[0].v1]
        else:
            return [tablero[0].v1, tablero[0].v2]
        
    nc1 = numeroDiferenteExtremo(tablero[0], tablero[1])
    nc2 = numeroDiferenteExtremo(tablero[-1], tablero[-2])
    return [nc1, nc2]


def jugadasDisponibles(tablero, piezas_robot):
    """Calcula todas las posibles piezas que el robot podria poner en cada extremo del tablero.
    
        Args:
            tablero (List[piezasencilla]): tablero ordenado de piezas.
            piezas_robot (List[piezasencilla]): piezas disponibles.
        Returns:
            posibles_jugadas_1 (List[piezasencilla]): piezas que podria poner en el primer extremo del tablero.
            posibles_jugadas_2 (List[piezasencilla]): piezas que podria poner en el segundo extremo del tablero.
    """
    extremos = extremosTablero(tablero)

    if len(extremos) == 0:
        return piezas_robot, piezas_robot
    
    posibles_jugadas_1 = []
    posibles_jugadas_2 = []

    for pieza in piezas_robot:
        if pieza.v1 == extremos[0] or pieza.v2 == extremos[0]:
            posibles_jugadas_1.append(pieza)
        if pieza.v1 == extremos[1] or pieza.v2 == extremos[1]:
            posibles_jugadas_2.append(pieza)
    
    return posibles_jugadas_1, posibles_jugadas_2

# solo para test, bastante inutil
def clasificarPiezas(piezas, alto_imagen, alto_zona_robot):
    """Esto lo uso para hacer algun test. Divide las piezas que hay arriba y las que hay abajo en dos, siendo las de arriba el tablero, y las de abajo las disponibles.
    
        Args:
            piezas (List[piezasencilla]): piezas.
            alto_imagen: alto (no necesariamente en pixeles de la imagen.
            alto_zona_robot: cuanto de la parte de abajo de la imagen son piezas disponibles.
        Returns:
            piezas_tablero (List[piezasencilla]): piezas del tablero.
            piezas_robot (List[piezasencilla]): piezas disponibles para el robot.
    """
    piezas_tablero= [] 
    piezas_robot = []

    for pieza in piezas:
        if alto_imagen - pieza.center[1] - 1 < alto_zona_robot:
            piezas_robot.append(pieza)
        else:
            piezas_tablero.append(pieza)

    return piezas_tablero, piezas_robot

# #######################################
# ######## FUNCIONES IMPORTANTES ########
# #######################################





def tableroVirtual(piezas, umbral_dist, orden):
    """Crea un tablero virtual. Para ello, ordena las piezas de la zona de juego, agrupandolas en una cadena de piezas contiguas.
    No comprueba que necesariamente las jugadas sean validas, solo las une en funcion de la distancia entre estas.
    La primera pieza siempre es la del extremo superior (o izquierdo).
    
        Args:
            piezas (List[piezasencilla]): piezas.
            umbral_dist: valor que indica cuanto de cerca tienen que estar los centros de las piezas para considerarse contiguas.
            orden:  orden de la norma para calcular las distancias.
        Returns:
            tablero (List[piezasencilla]): piezas ordenadas del tablero.
    """
    if len(piezas) == 1:
        return piezas
    elif len(piezas) == 2:
        if distanciaPiezas(piezas[0], piezas[1], orden) <= umbral_dist:
            return piezas
        else:
            return [piezas[0]]
        
    tablero = [piezas[0]]
    piezas_restantes = piezas[1:]

    # print([pieza.type for pieza in tablero])

    while piezas_restantes:
        ind, dist = masCercanos(tablero[-1], piezas_restantes, orden)
        if dist[ind[0]] <= umbral_dist:
            tablero.append(piezas_restantes[ind[0]])
            piezas_restantes.pop(ind[0])
            # print([pieza.type for pieza in tablero])
        else:
            break

    if piezas_restantes:
        tablero.reverse()
        while piezas_restantes:
            ind, dist = masCercanos(tablero[-1], piezas_restantes, orden)
            if dist[ind[0]] <= umbral_dist:
                tablero.append(piezas_restantes[ind[0]])
                piezas_restantes.pop(ind[0])
                # print([pieza.type for pieza in tablero])
            else:
                break

    # comprobar siempre rama superior o inferior
    if tablero[0].center[1] < tablero[-1].center[1] - umbral_dist / 2: 
        tablero.reverse() 
    elif tablero[0].center[1] < tablero[-1].center[1] + umbral_dist / 2:
        if tablero[0].center[0] < tablero[-1].center[0]:
            tablero.reverse()
    
    return tablero


def decidirMovimiento(tablero, piezas_robot): # aqui viene toda la IA :)
    """En funcion del tablero (conjunto ordenado de piezas contiguas que forman una partida de domino), decide la jugada que hacer.
    De momento, intenta usar la pieza de mayor valor posible.
    
        Args:
            tablero (List[piezasencilla]):     piezas ordenadas del tablero. La primera pieza siempre es la del extremo superior (o izquierdo).
            piezas_robot(List[piezasencilla]): piezas disponibles para el robot.
        Returns:
            accion (Dictionary): conjunto de ordenes que hay que jugar. Los valores son:
            -Comunes:
                'movimiento':       'jugada' si debe colocar una pieza. 'robar' si no hay pieza que pueda ser jugada.
            -Si movimientos['movimiento'] = 'jugada'
                'pieza_tablero':    pieza del tablero a continuacion de la cual el robot va a poner la suya.
                'pieza_robot':      pieza del robot que se desea jugar.
                'direccion':        en caso de que la ficha no se pueda colocar al lado debido a que se rebasan los limites del tablero, se debe colocar 'arriba' (extremo 1 del tablero) o 'abajo' (extremo dos del tablero).
    """
    posibles_jugadas1, posibles_jugadas2 = jugadasDisponibles(tablero, piezas_robot)

    if len(posibles_jugadas1): # hay jugadas posibles para la primera ficha del tablero
        #para elegir ficha de mayor valor
        suma_valores1 = [pieza.sumaValor() for pieza in posibles_jugadas1]
        ind_orden1 = sorted(range(len(suma_valores1)), key=lambda k: suma_valores1[k]) # stack overflow

        if len(posibles_jugadas2): # hay jugadas posibles para ambos extremos del tablero
            #para elegir ficha de mayor valor
            suma_valores2 = [pieza.sumaValor() for pieza in posibles_jugadas2]
            ind_orden2 = sorted(range(len(suma_valores2)), key=lambda k: suma_valores2[k]) # stack overflow

            if suma_valores1[ind_orden1[-1]] >= suma_valores2[ind_orden2[-1]]:
                return {'movimiento': 'jugada', 'pieza_tablero': tablero[0], 'pieza_robot': posibles_jugadas1[ind_orden1[-1]], 'direccion': 'abajo'}
            else:
                return {'movimiento': 'jugada', 'pieza_tablero': tablero[-1], 'pieza_robot': posibles_jugadas2[ind_orden2[-1]], 'direccion': 'arriba'}
        else:
            return {'movimiento': 'jugada', 'pieza_tablero': tablero[0], 'pieza_robot': posibles_jugadas1[ind_orden1[-1]], 'direccion': 'abajo'}
        
    elif len(posibles_jugadas2): # hay jugadas posibles para la ultima ficha del tablero pero no para la primera
        #para elegir ficha de mayor valor
        suma_valores2 = [pieza.sumaValor() for pieza in posibles_jugadas2]
        ind_orden2 = sorted(range(len(suma_valores2)), key=lambda k: suma_valores2[k]) # stack overflow
        return {'movimiento': 'jugada', 'pieza_tablero': tablero[-1], 'pieza_robot': posibles_jugadas2[ind_orden2[-1]], 'direccion': 'arriba'}
    
    else:
        return {'movimiento': 'robar'}

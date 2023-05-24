# import sys
# sys.path.append("..\vision")

from .colocar_pieza import colocarPieza
from .domino_game import tableroVirtual, decidirMovimiento
from .pieza_sencilla import PiezaSencilla, tablero2piezas, robot2piezas

def logica(valores_tablero, valores_robot):
    """Casi toda la logica. 
    A partir de las piezas que ahy jugadas sobre la mes (tablero), y las que tiene disponibles el robot, se decide la pieza a colocar y donde colocarla.
    Las piezas del tablero se indican como:
    [[pieza1.x, pieza1.y, pieza1.angulo, pieza1.valor1, pieza1.valor2], [pieza2.x, pieza2.y, pieza2.angulo, pieza2.valor1, pieza2.valor2], ...]
    Las piezas disponibles se indican como:
    [[hay_pieza_1, pieza1.valor1, pieza1.valor2], [hay_pieza_2, pieza2.valor1, pieza2.valor2], ...]
        Args:
            piezas_robot:    array con los valores de posicion, angulo y valor de las piezas del tablero.
            valores_robot:      array con los valores de las piezas, con un primer valor que indica si en esa posicion predefinida hay una pieza o no.
        Returns:
            List[]: [indice_pieza_a_colocar, x, y, angulo], con (x, y) posicion (en m) de donde queremos colocar la pieza, y el angulo (en grados) con el que se quiere colocar.
    """
     
    ORDEN_NORMA = 2
    # LONGITUD_PIEZA  = 80 # en píxeles
    # ANCHURA_PIEZA   = 40 # en píxeles  
    LONGITUD_PIEZA  = 0.038 # en m
    ANCHURA_PIEZA   = 0.019 # en m
    UMBRAL_DIST     = LONGITUD_PIEZA * 1.5
    # limites para no colocar en los extremos
    # LIMITE1 = 100
    # LIMITE2 = 540
    # LIMITE1 = .270 + .060 - .236/2 + .050
    # LIMITE2 = .270 + .060 + .236/2 - .050
    LIMITE2 = -(.270  - .314/2 + .050)
    LIMITE1 = -(.270  + .314/2 - .050)
    
    print(valores_tablero)
 
    # interpretar los arrays que me pasan y convertirlos en piezas
    piezas_robot = robot2piezas(valores_robot)
    piezas_tablero = tablero2piezas(valores_tablero)

    print("piezas disponibles: ")
    for pieza in piezas_robot:
        print(f"{[pieza.center[0], pieza.v1, pieza.v2]}")

    print("piezas del tablero: ")
    for pieza in piezas_tablero:
        print([pieza.center, pieza.v1, pieza.v2])
    
    for i in range(len(piezas_tablero)):
        new_x = -piezas_tablero[i].center[1] # -y
        new_y = -piezas_tablero[i].center[0] # -x
        piezas_tablero[i].center = [new_x, new_y]

 

    # Crear tablero virtual (ordenar las piezas)
    tablero = tableroVirtual(piezas_tablero, UMBRAL_DIST, ORDEN_NORMA)

    print("tablero: ")
    for pieza in tablero:
        print([pieza.v1, pieza.v2])

    # Decidir el movimiento
    movimiento = decidirMovimiento(tablero, piezas_robot)

    print("movimiento = ")
    print(movimiento['movimiento'])
    if movimiento['movimiento'] == 'jugada':
        print([movimiento['pieza_robot'].v1, movimiento['pieza_robot'].v2])
        print([movimiento['pieza_tablero'].v1, movimiento['pieza_tablero'].v2])   

    # Coordenadas del movimiento (origne, destino) -> coordenadas imagen
    origen, angulo_origen, destino, angulo_destino = colocarPieza(movimiento, LIMITE1, LIMITE2, LONGITUD_PIEZA, ANCHURA_PIEZA, tablero)

    return [origen[0], -destino[1], -destino[0], angulo_destino]
    # return [origen[0], destino[0], destino[1], angulo_destino]


def logica_test(piezas_tablero, piezas_robot):
    """Casi toda la logica. 
    A partir de las piezas que hay jugadas sobre la mes (tablero), y las que tiene disponibles el robot, se decide la pieza a colocar y donde colocarla.
    SOLO USAR PARA TEST
        Args:
            piezas_tablero (List[pieza_sencilla]):    piezas del tablero.
            piezas_robot (List[pieza_sencilla]):      piezas del robot, disponibles para jugar.
        Returns:
            List[]: [x_pieza_robot, y_pieza_robot, angulo_pieza_robot, x_destino, y_destino, angulo_destino], con (x, y) posicion (en pixeles) de la pieza a jugar o de donde queremos colocar la pieza, y el angulo (en grados) de la pieza y con el que se quiere colocar.
        """
    ORDEN_NORMA = 2
    LONGITUD_PIEZA  = 80 # en píxeles
    ANCHURA_PIEZA   = 40 # en píxeles  
    # LONGITUD_PIEZA  = 0.038 # en m
    # ANCHURA_PIEZA   = 0.019 # en m
    UMBRAL_DIST     = LONGITUD_PIEZA * 1.5
    # limites para no colocar en los extremos
    LIMITE1 = 100
    LIMITE2 = 540
    # LIMITE1 = 0.050
    # LIMITE2 = 0.0314 - 0.050

    # Crear tablero virtual (ordenar las piezas)
    tablero = tableroVirtual(piezas_tablero, UMBRAL_DIST, ORDEN_NORMA)

    # Decidir el movimiento
    movimiento = decidirMovimiento(tablero, piezas_robot)

    print("movimiento = ")
    print(movimiento['movimiento'])
    if movimiento['movimiento'] == 'jugada':
        print([movimiento['pieza_robot'].v1, movimiento['pieza_robot'].v2])
        print([movimiento['pieza_tablero'].v1, movimiento['pieza_tablero'].v2]) 

    # Coordenadas del movimiento (origne, destino) -> coordenadas imagen
    origen, angulo_origen, destino, angulo_destino = colocarPieza(movimiento, LIMITE1, LIMITE2, LONGITUD_PIEZA, ANCHURA_PIEZA, tablero)

    return [origen[0], origen[1], angulo_origen, destino[0], destino[1], angulo_destino]

class PiezaSencilla:
    def __init__(self, x = 0, y = 0, angle = 0, v1 = -1, v2 = -1):
        """Clase que define una pieza de domino mas sencilla, con menos cosas.

        Args: 
            x:      coordenada x del centro de la pieza.
            y:      coordenada y del centro de la pieza.
            angle:  angulo de la  pieza, en grados.
            v1:     primer valor de la pieza de domino (arriba o a la izquierda).
            v2:     segundo valor de la pieza de domino (abajo o a la derecha).
        """
        self.center = [x, y]
        self.angle = angle
        self.v1 = v1
        self.v2 = v2
          
    def esDoble(self):
        """La pieza es un doble.

        Args:
            piece (Piece): pieza

        Returns:
            bool: True: es un doble. False: no lo es.
        """
        if self.v1 == self.v2:
            return True
        else:
            return False
        
    def esVertical(self):
        """La pieza es más vertical que horizontal.

        Args:
            piece (Piece): pieza.

        Returns:
            bool: True: es vertical. False: es horizontal.
        """
        if abs(self.angle - 90) < 45:
            return True
        else:
            return False

    def esHorizontal(self):
        """La pieza es más horizontal que vertical.

        Args:
            piece (Piece): pieza

        Returns:
            bool: True: es horizontal. False: es vertical.
        """
        if abs(self.angle) < 45 or abs(self.angle - 180) < 45:
            return True
        else:
            return False

    def sumaValor(self):
        """Suma de los valores de la pieza, v1 + v2.

        Args:
            piece (Piece): pieza.

        Returns:
            int: Valor total de la pieza.
        """
        return int(self.v1) + int(self.v2)
    

def tablero2piezas(valores_tablero):
    """Convierte un array del siguiente tipo en un array de piezas sencillas.
    [[pieza1.x, pieza1.y, pieza1.angulo, pieza1.valor1, pieza1.valor2], [pieza2.x, pieza2.y, pieza2.angulo, pieza2.valor1, pieza2.valor2], ...]

    Args:
        valores_tablero: array con los valores de posicion, angulo y valor de las piezas del tablero.

    Returns:
        piezas: piezas sencillas del tablero.
    """
    piezas = []
    for valores in valores_tablero:
        pieza = PiezaSencilla(x=valores[0], y=valores[1], angle=valores[2], v1=valores[3], v2=valores[4])
        piezas.append(pieza)
    return piezas

def robot2piezas(valores_robot):
    """Convierte un array del siguiente tipo en un array de piezas sencillas.
    [[hay_pieza_1, pieza1.valor1, pieza1.valor2], [hay_pieza_2, pieza2.valor1, pieza2.valor2], ...]

    Args:
        valores_robot: array con los valores de las piezas, con un primer valor que indica si en esa posicion predefinida hay una pieza o no.

    Returns:
        piezas: piezas sencillas que posee el robot. Su posicion x indica la posicion predefinida en la que se encuentra.
    """
    piezas = []
    for i in range(len(valores_robot)):
        if valores_robot[i][0]:
            pieza = PiezaSencilla(x=i, y=0, angle=0, v1=valores_robot[i][1], v2=valores_robot[i][2])
            piezas.append(pieza)
    return piezas



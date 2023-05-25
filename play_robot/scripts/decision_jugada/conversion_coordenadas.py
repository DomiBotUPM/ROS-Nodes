def conversionCoordenadasJuego(x_pieza_coord_img, y_pieza_coord_img, theta_pieza_coord_img, x_gripper_abs = 270, y_gripper_abs = 196, alto_zona_juego = 236, ancho_zona_juego = 314, x_camara_resp_gripper = 50, y_camara_resp_gripper = -22):
    """Conversion de coordenadas de la pieza en la imagen (en mm, no en pixeles) a las coordenadas absolutas de la pieza, respecto de la base del robot.
        Args:
            x_pieza_coord_img:      coordenada x de la pieza respecto de la esquina superior izquierda de la imagen, en mm
            y_pieza_coord_img:      coordenada y de la pieza respecto de la esquina superior izquierda de la imagen, en mm
            theta_pieza_coord_img:  angulo de la pieza segun se ve en la imagen (entre -45 y 135, o entre 0 y 180)
            x_gripper_abs:          coordenada x absoluta del gripper (respecto de la base del robot), en mm
            y_gripper_abs:          coordenada y absoluta del gripper (respecto de la base del robot), en mm
            alto_zona_juego:        alto (height) de la zona de juego, segun se ve en la camara, en mm
            ancho_zona_juego:       ancho (width) de la zona de juego, segun se ve en la camara, en mm
            x_camara_resp_gripper:  coordenada x de la camara respecto del gripper, en mm
            y_camara_resp_gripper:  coordenada y de la camara respecto del gripper, en mm
        Returns:
            x_pieza_abs:            coordenada x absoluta de la pieza (respecto de la base del robot), en mm
            y_pieza_abs:            coordenada y absoluta de la pieza (respecto de la base del robot), en mm
            theta_pieza_coord_img:  angulo de la pieza absoluto (respecto de la base del robot)
        """


    x_pieza_resp_camara = alto_zona_juego - y_pieza_coord_img       # cambio de coordenadas x, y - coordenada x de la pieza respecto de la camara
    y_pieza_resp_camara = ancho_zona_juego - x_pieza_coord_img      # cambio de coordenadas x, y - coordenada y de la pieza respecto de la camara
    x_pieza_abs = x_pieza_resp_camara - alto_zona_juego / 2 + x_camara_resp_gripper + x_gripper_abs
    y_pieza_abs = y_pieza_resp_camara - alto_zona_juego / 2 + y_camara_resp_gripper + y_gripper_abs

    # la parte de por aquí ya pensamos que hacer
    # if abs(theta_pieza_coord_img) < 45 or abs(theta_pieza_coord_img - 180) < 45:
    #     theta_pieza_abs = 90
    # else:
    #     theta_pieza_abs = 0
        
    theta_pieza_abs = theta_pieza_coord_img
        
    return [x_pieza_abs, y_pieza_abs, theta_pieza_abs]

## 0.35 0.32 rz = 1.992 rad +- 0.3
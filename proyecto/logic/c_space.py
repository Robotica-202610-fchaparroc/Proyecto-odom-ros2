import numpy as np
from .point_poligone import punto_dentro_o_frontera_poligono
from .config_data import EPS

def calcular_limites_cspace(ancho, alto, robot_rotado):
    x_min_robot = np.min(robot_rotado[:, 0])
    x_max_robot = np.max(robot_rotado[:, 0])
    y_min_robot = np.min(robot_rotado[:, 1])
    y_max_robot = np.max(robot_rotado[:, 1])

    c_x_min = -x_min_robot
    c_x_max = ancho - x_max_robot
    c_y_min = -y_min_robot
    c_y_max = alto - y_max_robot

    if abs(c_x_min) < EPS:
        c_x_min = 0.0
    if abs(c_y_min) < EPS:
        c_y_min = 0.0

    return c_x_min, c_x_max, c_y_min, c_y_max

def punto_en_frontera_cspace(punto, c_x_min, c_x_max, c_y_min, c_y_max, eps=EPS):
    x, y = punto
    return (
        abs(x - c_x_min) <= eps or
        abs(x - c_x_max) <= eps or
        abs(y - c_y_min) <= eps or
        abs(y - c_y_max) <= eps
    )

def punto_invalido(punto, c_obstaculo, c_x_min, c_x_max, c_y_min, c_y_max):
    x = punto[0]
    y = punto[1]

    fuera_c = (x < c_x_min or x > c_x_max or y < c_y_min or y > c_y_max)
    en_frontera_c = punto_en_frontera_cspace(punto, c_x_min, c_x_max, c_y_min, c_y_max)
    dentro_o_frontera_obstaculo = punto_dentro_o_frontera_poligono(punto, c_obstaculo)

    return fuera_c or en_frontera_c or dentro_o_frontera_obstaculo

def buscar_celda_punto_cspace(self, x, y, matriz, dx, dy, ancho, alto):
    columnas = int(round(ancho / dx))
    filas = int(round(alto / dy))

    if not (0.0 <= x <= ancho and 0.0 <= y <= alto):
        return None, None, "FUERA"

    columna = min(int(x / dx), columnas - 1)
    fila_desde_abajo = min(int(y / dy), filas - 1)
    fila_desde_arriba = filas - 1 - fila_desde_abajo

    estado = matriz[fila_desde_arriba][columna]
    return fila_desde_arriba, columna, estado

def nombre_estado_celda(self, estado):
    nombres = {
        "L": "Libre",
        "S": "Semilibre",
        "O": "Ocupada",
        "FUERA": "Fuera del workspace",
    }
    return nombres.get(estado, estado)